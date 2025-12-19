import os
import json
import time
from datetime import datetime
from typing import Any
from uuid import UUID, uuid4
import shutil
import threading
import copy

from item_utility import get_item_size, get_folder_size
import glob


class Catalog:
    TIME_FORMAT = "%y%m%d_%H%M%S_%f"

    def __init__(self, library_path: str):
        self.data_path = os.path.join(library_path, "data")
        self.json_path = os.path.join(library_path, "catalog.json")
        self.data = {}

        self.open_catalog()

        self._write_lock = threading.Lock()
        self._dirty = threading.Event()
        self._exit_worker = threading.Event()

        self._flush_worker_thread = threading.Thread(target=self._flush_worker)
        self._flush_worker_thread.start()

    def now(self):
        return datetime.now().strftime(Catalog.TIME_FORMAT)

    def open_catalog(self):
        try:
            with open(self.json_path, "r") as f:
                self.data = json.load(f)
        except (json.JSONDecodeError, IOError, FileNotFoundError):
            self._initialize_empty_catalog()

    def close(self):
        self._exit_worker.set()
        self._dirty.set()
        self._flush_worker_thread.join()

    def _flush_worker(self):
        """Worker thread function to save data to disk when dirty."""
        while not self._exit_worker.is_set():
            if self._dirty.wait(timeout=1):
                if self._exit_worker.is_set():
                    break

                with self._write_lock:
                    data_copy = copy.deepcopy(self.data)
                    self._dirty.clear()

                data_copy["m_time"] = self.now()
                with open(self.json_path, "w") as f:
                    json.dump(data_copy, f, indent=4)

    def _flush_catalog(self):
        """signals the worker thread to save the data"""
        self._dirty.set()

    def _initialize_empty_catalog(self):
        os.makedirs(self.data_path, exist_ok=True)
        self.data = {
            "c_time": self.now(),
            "m_time": self.now(),
            "storage_stats": {
                "Photos": 0,
                "Videos": 0,
                "Scans": 0
            },
            "items": {},
        }

    def _ensure_storage_stats(self):
        """Ensure storage_stats exists in data."""
        if "storage_stats" not in self.data:
            self.data["storage_stats"] = {
                "Photos": 0,
                "Videos": 0,
                "Scans": 0
            }

    def get_storage_stats(self):
        """Return storage stats."""
        self._ensure_storage_stats()
        return self.data["storage_stats"]

    def _get_item_category(self, item_type: str) -> str:
        """Map item type to storage category."""
        if "Image" in item_type:
            return "Photos"
        elif "Video" in item_type:
            return "Videos"
        elif "Scan" in item_type or "Map" in item_type:
            return "Scans"
        return None

    def _update_storage_stats(self, item_type: str, size_delta: int):
        """Update storage stats for a category."""
        self._ensure_storage_stats()
        category = self._get_item_category(item_type)
        if category:
            self.data["storage_stats"][category] = max(
                0, self.data["storage_stats"].get(category, 0) + size_delta
            )

    def _get_new_item_name(self):
        return self.now()

    def _get_prefixed_item_name(self, prefix: str) -> str:
        """
        Generate a unique item name with prefix and incremental index.
        E.g., "Scan" -> "Scan_1", "Scan_2", etc.
        """
        # Find all existing items with this prefix
        existing_names = [
            item["name"] for item in self.data.get("items", {}).values()
        ]

        # Find the highest index used with this prefix
        max_index = 0
        prefix_pattern = f"{prefix}_"
        for name in existing_names:
            if name.startswith(prefix_pattern):
                try:
                    suffix = name[len(prefix_pattern):]
                    index = int(suffix)
                    max_index = max(max_index, index)
                except ValueError:
                    continue

        # Return next index
        return f"{prefix}_{max_index + 1}"

    def get_data(self):
        return self.data

    def add_item(self, item_type: str, content_path: str, name_prefix: str = None):
        """
        Adds a new item to the catalog library and update catalog.json
        Args:
            item_type: Type of the item (e.g., "Image", "Video", "Scan")
            content_path: path to the folder containing item files
            name_prefix: Optional prefix for item name. If provided, generates
                         names like "prefix_1", "prefix_2". If None, uses timestamp.
        """
        if name_prefix:
            item_name = self._get_prefixed_item_name(name_prefix)
        else:
            item_name = self._get_new_item_name()
        item_uuid = str(uuid4())
        item_disk_path = os.path.join(self.data_path, item_name)
        # Ensure parent directory exists, but NOT item_disk_path itself
        # so shutil.move renames content_path to item_disk_path
        # os.makedirs(self.data_path, exist_ok=True)

        shutil.move(content_path, item_disk_path)

        # Calculate metadata for the new item
        size_on_disk = get_folder_size(item_disk_path)
        item_size = get_item_size(item_type, item_disk_path)

        with self._write_lock:
            self.data["items"][item_uuid] = {
                "name": item_name,
                "type": item_type,
                "url": item_disk_path,
                "date": time.time(),
                "size_on_disk": size_on_disk,
                "size": item_size
            }

        # Update storage stats
        self._update_storage_stats(item_type, size_on_disk)

        self._flush_catalog()
        return item_uuid

    def remove_item(self, item_uuid: str):
        """Removes items from the catalog by their UUIDs."""
        with self._write_lock:
            items_dict = self.data.get("items", {})
            if item_uuid in items_dict:
                item_data = items_dict.pop(item_uuid)
                item_disk_path = item_data["url"]
                # Update storage stats (decrement)
                item_type = item_data.get("type", "")
                size_on_disk = item_data.get("size_on_disk", 0)
                self._update_storage_stats(item_type, -size_on_disk)
                if os.path.isdir(item_disk_path):
                    shutil.rmtree(item_disk_path)
                self._flush_catalog()

    def clear_all_items(self):
        """Removes all items from the catalog (for formatting internal storage)."""
        with self._write_lock:
            items_dict = self.data.get("items", {})
            # Delete all item directories
            for item_data in items_dict.values():
                item_disk_path = item_data.get("url", "")
                if os.path.isdir(item_disk_path):
                    shutil.rmtree(item_disk_path)
            # Clear items and reset storage stats
            self.data["items"] = {}
            self.data["storage_stats"] = {
                "Photos": 0,
                "Videos": 0,
                "Scans": 0
            }
            self._flush_catalog()

    def _safe_path(self, path: str) -> str:
        tries = 1
        new_path = path
        while (os.path.exists(new_path)):
            new_path = f"{path}_{tries}"
            tries += 1
        return new_path

    def rename_item(self, item_uuid: str, new_name: str):
        """Renames an item in the catalog by its UUID."""
        with self._write_lock:
            items_dict = self.data.get("items", {})
            if item_uuid in items_dict:
                item_data = items_dict[item_uuid]
                item_disk_path = item_data["url"]
                new_disk_path = os.path.join(
                    os.path.dirname(item_disk_path), new_name)
                new_disk_path = self._safe_path(new_disk_path)
                shutil.move(item_disk_path, new_disk_path)
                item_data["name"] = new_name
                item_data["url"] = new_disk_path
                self._flush_catalog()
                return item_data
            else:
                raise ValueError(f"Item with UUID {item_uuid} not found")

    def export_item(self, item_uuid: str, mountpoint: str) -> str:
        """
        Exports an item to a removable storage device.
        Adds task to queue, returns task_id for tracking.
        """
        from export_manager import export_manager

        items_dict = self.data.get("items", {})
        if item_uuid not in items_dict:
            raise ValueError(f"Item with UUID {item_uuid} not found")

        item_data = items_dict[item_uuid]
        item_disk_path = item_data["url"]
        item_name = item_data["name"]
        target_directory = os.path.join(mountpoint, item_name)

        if os.path.exists(target_directory):
            raise RuntimeError(f"Item '{item_name}' already exported")

        # Add task to queue (will be processed sequentially)
        task_id = export_manager.add_task(
            item_id=item_uuid,
            item_name=item_name,
            source_path=item_disk_path,
            dest_path=target_directory
        )

        return task_id
