"""
Integration tests for Export API endpoints.

Tests the export queue management endpoints:
- GET /export/progress - Get overall export progress
"""

import pytest
from httpx import AsyncClient
from unittest.mock import patch, MagicMock
from export_manager import ExportManager, ExportTask, TaskStatus


@pytest.mark.asyncio
@pytest.mark.integration
async def test_get_export_progress_empty_queue(async_client: AsyncClient):
    """
    Test GET /export/progress with no exports returns empty queue state.

    Expected response:
    {
      "total_tasks": 0,
      "completed_tasks": 0,
      "failed_tasks": 0,
      "queued_tasks": 0,
      "overall_progress": 0.0,
      "current_item": null,
      "is_complete": true
    }
    """
    # Ensure export manager is clean
    from export_manager import export_manager
    export_manager.clear_all_tasks()

    # Get export progress with empty queue
    response = await async_client.get("/export/progress")

    # Verify response
    assert response.status_code == 200
    data = response.json()

    assert data["total_tasks"] == 0
    assert data["completed_tasks"] == 0
    assert data["failed_tasks"] == 0
    assert data["queued_tasks"] == 0
    assert data["overall_progress"] == 0.0
    assert data["current_item"] is None
    assert data["is_complete"] is True


@pytest.mark.asyncio
@pytest.mark.integration
async def test_get_export_progress_active(async_client: AsyncClient):
    """
    Test GET /export/progress with 3 items queued, 1st at 50% progress.

    Expected:
    - overall_progress: 16-17% (50% of 1/3)
    - current_item with progress: 50
    - queued_tasks: 2
    """
    from export_manager import export_manager

    # Clean export manager
    export_manager.clear_all_tasks()

    # Create 3 tasks with equal size (1000 bytes each)
    task_size = 1000

    # Manually create tasks to have precise control
    with export_manager._lock:
        # Task 1: Currently copying at 50%
        task1 = ExportTask(
            task_id="task-1",
            item_id="item-1",
            item_name="Item1",
            source_path="/fake/source1",
            dest_path="/fake/dest1",
            total_size=task_size,
            current_progress=50.0,
            status=TaskStatus.COPYING
        )
        export_manager.tasks["task-1"] = task1

        # Task 2: Queued
        task2 = ExportTask(
            task_id="task-2",
            item_id="item-2",
            item_name="Item2",
            source_path="/fake/source2",
            dest_path="/fake/dest2",
            total_size=task_size,
            current_progress=0.0,
            status=TaskStatus.QUEUED
        )
        export_manager.tasks["task-2"] = task2

        # Task 3: Queued
        task3 = ExportTask(
            task_id="task-3",
            item_id="item-3",
            item_name="Item3",
            source_path="/fake/source3",
            dest_path="/fake/dest3",
            total_size=task_size,
            current_progress=0.0,
            status=TaskStatus.QUEUED
        )
        export_manager.tasks["task-3"] = task3

        # Set total queue size
        export_manager._total_queue_size = task_size * 3

    # Get export progress
    response = await async_client.get("/export/progress")

    # Verify response
    assert response.status_code == 200
    data = response.json()

    # Verify task counts
    assert data["total_tasks"] == 3
    assert data["completed_tasks"] == 0
    assert data["failed_tasks"] == 0
    assert data["queued_tasks"] == 2

    # Verify overall progress: 50% of 1/3 = 16.666...%
    assert 16.0 <= data["overall_progress"] <= 17.0

    # Verify current item
    assert data["current_item"] is not None
    assert data["current_item"]["name"] == "Item1"
    assert data["current_item"]["progress"] == 50.0

    # Not complete while tasks are still queued/copying
    assert data["is_complete"] is False

    # Cleanup
    export_manager.clear_all_tasks()
