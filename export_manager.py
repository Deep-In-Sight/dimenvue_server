"""
Export queue manager with rsync progress tracking
"""
import os
import subprocess
import threading
import queue
import time
from typing import Dict, Optional
from dataclasses import dataclass, field
from enum import Enum
from uuid import uuid4


class TaskStatus(Enum):
    QUEUED = "queued"
    COPYING = "copying"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass
class ExportTask:
    task_id: str
    item_id: str
    item_name: str
    source_path: str
    dest_path: str
    total_size: int
    current_progress: float = 0.0  # 0-100%
    status: TaskStatus = TaskStatus.QUEUED
    error: Optional[str] = None
    start_time: Optional[float] = None
    end_time: Optional[float] = None


class ExportManager:
    def __init__(self):
        self.task_queue = queue.Queue()
        self.tasks: Dict[str, ExportTask] = {}
        self._lock = threading.Lock()
        self._worker_thread = None
        self._stop_worker = False
        self._current_task_id = None
        self._total_queue_size = 0

        # Start worker thread
        self._start_worker()

    def _start_worker(self):
        """Start the worker thread that processes queue"""
        self._stop_worker = False
        self._worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker_thread.start()

    def _worker_loop(self):
        """Continuously process tasks from queue one at a time"""
        while not self._stop_worker:
            try:
                # Get next task from queue (blocks with timeout)
                task_id = self.task_queue.get(timeout=1.0)

                with self._lock:
                    self._current_task_id = task_id
                    task = self.tasks.get(task_id)

                if task:
                    self._execute_task(task)

                with self._lock:
                    self._current_task_id = None

                self.task_queue.task_done()
            except queue.Empty:
                continue

    def add_task(self, item_id: str, item_name: str, source_path: str, dest_path: str) -> str:
        """
        Add export task to queue
        Returns task_id
        """
        task_id = str(uuid4())

        # Calculate item size
        total_size = self._get_directory_size(source_path)

        task = ExportTask(
            task_id=task_id,
            item_id=item_id,
            item_name=item_name,
            source_path=source_path,
            dest_path=dest_path,
            total_size=total_size
        )

        with self._lock:
            self.tasks[task_id] = task
            self._total_queue_size += total_size

        # Add to queue
        self.task_queue.put(task_id)

        return task_id

    def _execute_task(self, task: ExportTask):
        """Execute single export task using rsync"""
        try:
            task.status = TaskStatus.COPYING
            task.start_time = time.time()

            # Use rsync with progress
            self._rsync_with_progress(task)

            task.status = TaskStatus.COMPLETED
            task.current_progress = 100.0
            task.end_time = time.time()

            # If this is the last task, sync to disk
            if self._is_last_task(task):
                os.sync()

        except Exception as e:
            task.status = TaskStatus.FAILED
            task.error = str(e)
            task.end_time = time.time()

    def _is_last_task(self, task: ExportTask) -> bool:
        """Check if this is the last task to complete"""
        with self._lock:
            # Check if queue is empty and no other tasks are pending/copying
            is_queue_empty = self.task_queue.empty()
            other_active = any(
                t.task_id != task.task_id and t.status in (TaskStatus.QUEUED, TaskStatus.COPYING)
                for t in self.tasks.values()
            )
            return is_queue_empty and not other_active

    def _rsync_with_progress(self, task: ExportTask):
        """Copy using rsync and track progress"""
        source = task.source_path
        dest = task.dest_path

        # Ensure source ends with / for rsync
        if not source.endswith('/'):
            source += '/'

        # Create parent directory
        os.makedirs(os.path.dirname(dest), exist_ok=True)

        # rsync command with progress
        cmd = [
            'rsync',
            '-ah',  # archive, human-readable
            '--info=progress2',  # overall progress
            source,
            dest
        ]

        try:
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                bufsize=1
            )

            # Parse progress from rsync output
            for line in process.stdout:
                line = line.strip()
                if '%' in line:
                    # Parse progress percentage
                    # Format: "     123,456  12%   1.23MB/s    0:00:05"
                    parts = line.split()
                    for part in parts:
                        if part.endswith('%'):
                            try:
                                progress = float(part.rstrip('%'))
                                task.current_progress = min(progress, 100.0)
                            except ValueError:
                                pass
                            break

            process.wait()

            if process.returncode != 0:
                stderr = process.stderr.read()
                raise RuntimeError(f"rsync failed: {stderr}")

        except FileNotFoundError:
            # rsync not available, fallback to shutil
            import shutil
            shutil.copytree(source.rstrip('/'), dest, dirs_exist_ok=True)
            task.current_progress = 100.0

    def _get_directory_size(self, path: str) -> int:
        """Calculate total size of directory"""
        total_size = 0
        for root, dirs, files in os.walk(path):
            for file_name in files:
                file_path = os.path.join(root, file_name)
                try:
                    total_size += os.path.getsize(file_path)
                except OSError:
                    pass
        return total_size

    def get_overall_progress(self) -> Dict:
        """Get overall progress across all tasks in queue"""
        with self._lock:
            all_tasks = list(self.tasks.values())
            total_queue_size = self._total_queue_size

        if not all_tasks or total_queue_size == 0:
            return {
                'total_tasks': 0,
                'completed_tasks': 0,
                'failed_tasks': 0,
                'queued_tasks': 0,
                'overall_progress': 0.0,
                'current_item': None,
                'is_complete': True
            }

        total_tasks = len(all_tasks)
        completed_tasks = sum(1 for t in all_tasks if t.status == TaskStatus.COMPLETED)
        failed_tasks = sum(1 for t in all_tasks if t.status == TaskStatus.FAILED)
        queued_tasks = sum(1 for t in all_tasks if t.status == TaskStatus.QUEUED)

        # Calculate overall progress: sum(item_size * item_progress%) / total_queue_size
        progress_sum = 0.0
        for task in all_tasks:
            if task.status == TaskStatus.COMPLETED:
                progress_sum += task.total_size * 100.0
            elif task.status == TaskStatus.COPYING:
                progress_sum += task.total_size * task.current_progress
            # QUEUED and FAILED contribute 0

        overall_progress = (progress_sum / total_queue_size) if total_queue_size > 0 else 0.0

        # Get current copying item
        current_item = None
        copying_tasks = [t for t in all_tasks if t.status == TaskStatus.COPYING]
        if copying_tasks:
            task = copying_tasks[0]
            current_item = {
                'name': task.item_name,
                'progress': task.current_progress
            }

        is_complete = (queued_tasks == 0) and (copying_tasks == [])

        return {
            'total_tasks': total_tasks,
            'completed_tasks': completed_tasks,
            'failed_tasks': failed_tasks,
            'queued_tasks': queued_tasks,
            'overall_progress': overall_progress,
            'current_item': current_item,
            'is_complete': is_complete
        }

    def get_task_results(self) -> Dict:
        """Get results of all completed tasks"""
        with self._lock:
            all_tasks = list(self.tasks.values())

        succeeded = [t.item_name for t in all_tasks if t.status == TaskStatus.COMPLETED]
        failed = [
            {'name': t.item_name, 'reason': t.error or 'Copy failed'}
            for t in all_tasks if t.status == TaskStatus.FAILED
        ]

        return {
            'succeeded': succeeded,
            'failed': failed
        }

    def clear_all_tasks(self):
        """Clear all tasks (start new export session)"""
        with self._lock:
            # Clear queue
            while not self.task_queue.empty():
                try:
                    self.task_queue.get_nowait()
                    self.task_queue.task_done()
                except queue.Empty:
                    break

            # Clear tasks
            self.tasks.clear()
            self._total_queue_size = 0
            self._current_task_id = None

    def stop(self):
        """Stop the worker thread"""
        self._stop_worker = True
        if self._worker_thread:
            self._worker_thread.join(timeout=5.0)


# Global export manager instance
export_manager = ExportManager()
