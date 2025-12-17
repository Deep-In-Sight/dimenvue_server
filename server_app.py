import os
from pathlib import Path
from typing import Optional, Any
from contextlib import asynccontextmanager

from uuid import UUID, uuid4
from fastapi import FastAPI, HTTPException, Body
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from camera_app import MultiCamApp
from mapping_app import MappingApp
from settings import Settings
from catalog import Catalog
from item_utility import get_metadata_factory
import time
from storage import get_removable, get_removable_dummy, get_internal_usage, format_internal, format_external
from export_manager import export_manager

OUTPUT_DIR: Path = Path(os.getenv("CAMERA_APP_OUTPUT_DIR", "/dmv_data"))
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

settings = Settings(OUTPUT_DIR / "settings.json")
catalog = Catalog(library_path=str(OUTPUT_DIR / "library"))
cameraApp = MultiCamApp(media_path=str(OUTPUT_DIR), catalog=catalog)
mappingApp = MappingApp(data_path=str(OUTPUT_DIR), catalog=catalog)


@asynccontextmanager
async def lifespan(app: FastAPI):
    global cameraApp, mappingApp, catalog, settings
    yield
    catalog.close()
    del cameraApp, mappingApp, catalog, settings

# slamApp = SlamApp()
app = FastAPI(lifespan=lifespan)

# Serve generated artifacts for convenience
app.mount("/dmv_data", StaticFiles(directory=str(OUTPUT_DIR)))


@app.get("/catalog")
def get_catalog():
    return catalog.get_data()


@app.get("/catalog/metadata/{url:path}")
def get_metadata(url: str):
    try:
        metadata = get_metadata_factory(url)
        return metadata
    except (ValueError, FileNotFoundError) as e:
        raise HTTPException(status_code=404, detail=str(e))


@app.put("/catalog/{uuid:uuid}/delete")
def delete_item(uuid: UUID):
    print(f"delete_item: {uuid}")
    catalog.remove_item(str(uuid))
    return {}


@app.put("/catalog/{uuid:uuid}/rename")
def rename_item(uuid: UUID, req: dict[str, Any]):
    try:
        name = req["name"]
        print(f"rename_item: {uuid} to {name}")
        new_item = catalog.rename_item(str(uuid), name)
        return new_item
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))


@app.put("/cameraApp/init")
async def cameraApp_init():
    """
    Initialize camera GStreamer pipelines.

    Returns:
        200: Initialization successful or already initialized
        409: Camera is in a transitional state (initializing/deinitializing)
        500: Initialization failed
    """
    try:
        await cameraApp.gst_init()
    except RuntimeError as e:
        # Camera is in invalid state for init (e.g., already initializing)
        raise HTTPException(status_code=409, detail=str(e))
    except Exception as e:
        # Unexpected error during initialization
        print(f"init error: {e}")
        raise HTTPException(status_code=500, detail=f"Initialization failed: {str(e)}")


@app.put("/cameraApp/deinit")
async def cameraApp_deinit():
    """
    Deinitialize camera GStreamer pipelines.

    Returns:
        200: Deinitialization successful or already deinitialized
        409: Camera is in a transitional state (initializing/deinitializing)
        500: Deinitialization failed
    """
    try:
        await cameraApp.gst_deinit()
    except RuntimeError as e:
        # Camera is in invalid state for deinit (e.g., already deinitializing)
        raise HTTPException(status_code=409, detail=str(e))
    except Exception as e:
        # Unexpected error during deinitialization
        print(f"deinit error: {e}")
        raise HTTPException(status_code=500, detail=f"Deinitialization failed: {str(e)}")


@app.get("/cameraApp/state")
def get_camera_state():
    """
    Get current camera initialization state.

    Returns:
        dict: Current state, initialization status, and error information
    """
    return cameraApp.get_state()


@app.get("/cameraApp/preview-index")
def get_preview_index():
    """Get current preview index."""
    return {"index": cameraApp.current_preview_index}


@app.put("/cameraApp/preview-index")
def set_preview_index(req: dict[str, Any]):
    index = req["index"]
    try:
        active_index = cameraApp.setPreviewIndex(index)
        resp = {"index": active_index}
        print(f"set_preview_index: {resp}")
        return resp
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.put("/cameraApp/capture")
async def capture():
    """
    Capture a frame from all cameras.

    Returns:
        200: Capture successful
        409: Camera is in a state that doesn't allow capture
        500: Capture failed
    """
    try:
        await cameraApp.capture()
        resp = {"status": "captured"}
        print(f"capture: {resp}")
        return resp
    except RuntimeError as e:
        # Camera is in invalid state for capture
        raise HTTPException(status_code=409, detail=str(e))
    except Exception as e:
        # Unexpected error during capture
        print(f"capture error: {e}")
        raise HTTPException(status_code=500, detail=f"Capture failed: {str(e)}")


@app.put("/cameraApp/recordStart")
async def record_start():
    """
    Start recording from all cameras.

    Returns:
        200: Recording started
        409: Camera is in a state that doesn't allow recording
        500: Recording start failed
    """
    try:
        await cameraApp.recordStart()
        resp = {"status": "recording_started"}
        print(f"record_start: {resp}")
        return resp
    except RuntimeError as e:
        # Camera is in invalid state for recording
        raise HTTPException(status_code=409, detail=str(e))
    except Exception as e:
        # Unexpected error during recording start
        print(f"record_start error: {e}")
        raise HTTPException(status_code=500, detail=f"Recording start failed: {str(e)}")


@app.put("/cameraApp/recordStop")
async def record_stop():
    """
    Stop recording and save the video.

    Returns:
        200: Recording stopped
        409: Camera is not currently recording
        500: Recording stop failed
    """
    try:
        await cameraApp.recordStop()
        resp = {"status": "recording_stopped"}
        print(f"record_stop: {resp}")
        return resp
    except RuntimeError as e:
        # Camera is in invalid state for stopping
        raise HTTPException(status_code=409, detail=str(e))
    except Exception as e:
        # Unexpected error during recording stop
        print(f"record_stop error: {e}")
        raise HTTPException(status_code=500, detail=f"Recording stop failed: {str(e)}")


@app.get("/cameraApp/settings")
def get_camera_settings():
    return cameraApp.get_settings()


@app.put("/cameraApp/settings")
def put_camera_settings(req: dict[str, Any]):
    new_settings = req.get("settings", req)
    try:
        return cameraApp.save_settings(new_settings)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.get("/mappingApp/settings")
def get_mapping_settings():
    return mappingApp.get_settings()


@app.put("/mappingApp/settings")
def put_mapping_settings(req: dict[str, Any]):
    new_settings = req.get("settings", req)
    return mappingApp.save_settings(new_settings)


@app.get("/storage/internal/usage")
def get_internal_storage_usage():
    return get_internal_usage(catalog)


@app.put("/storage/internal/format")
def format_internal_storage():
    return format_internal(catalog)


@app.put("/storage/external/format")
def format_external_storage(req: dict[str, Any]):
    try:
        mountpoint = req["mountpoint"]
        return format_external(mountpoint)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.get("/storage/removable")
def get_removable_list():
    return get_removable()


@app.put("/catalog/{uuid:uuid}/export")
def export_item(uuid: UUID, req: dict[str, Any]):
    """Export a single item"""
    try:
        mountpoint = req["mountpoint"]
        item_uuid = str(uuid)

        print(f"export_item: {item_uuid} to {mountpoint}")

        # Create and start export task via catalog
        task_id = catalog.export_item(item_uuid, mountpoint)

        return {"task_id": task_id}
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except RuntimeError as e:
        print(f"export_item: error: {e}")
        raise HTTPException(status_code=409, detail=str(e))


@app.get("/export/progress")
def get_overall_export_progress():
    """Get overall progress of all export tasks in queue"""
    return export_manager.get_overall_progress()


@app.get("/export/results")
def get_export_results():
    """Get results of all completed export tasks"""
    return export_manager.get_task_results()


@app.post("/export/clear")
def clear_export_queue():
    """Clear all export tasks (start new export session)"""
    export_manager.clear_all_tasks()
    return {"status": "cleared"}


@app.put("/mappingApp/start")
async def start_mapping():
    """
    Start the mapping process by launching ROS2 Fast-LIO.

    Returns:
        200: Mapping started or already running
        409: Mapping is in a transitional state (starting/stopping)
        500: Failed to start mapping
    """
    try:
        resp = await mappingApp.start()
        return resp
    except RuntimeError as e:
        # Mapping is in invalid state for start
        raise HTTPException(status_code=409, detail=str(e))
    except Exception as e:
        # Unexpected error during start
        print(f"start_mapping error: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to start mapping: {str(e)}")


@app.get("/mappingApp/state")
def get_mapping_state():
    """
    Get current mapping state for status monitoring.

    Returns:
        dict: Current state, process info, and error information
    """
    return mappingApp.get_state()


@app.put("/mappingApp/stop")
async def stop_mapping():
    """
    Stop the mapping process and finalize results.

    Returns:
        200: Mapping stopped or already stopped
        409: Mapping is in a transitional state (starting/stopping)
        500: Failed to stop mapping
    """
    try:
        resp = await mappingApp.stop()
        return resp
    except RuntimeError as e:
        # Mapping is in invalid state for stop
        raise HTTPException(status_code=409, detail=str(e))
    except Exception as e:
        # Unexpected error during stop
        print(f"stop_mapping error: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to stop mapping: {str(e)}")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
