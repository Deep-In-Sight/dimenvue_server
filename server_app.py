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


@app.put("/initEverything")
def init_everything():
    resp = {}
    print(f"init_everything: {resp}")
    time.sleep(2)
    return resp


@app.get("/settings")
def get_all_settings():
    return settings.data


@app.get("/settings/{settingKey:path}")
def get_settings(settingKey: str):
    try:
        value = settings.get(settingKey)
        print(f"get_settings: {value}")
        return {"value": value}
    except KeyError:
        raise HTTPException(
            status_code=404, detail=f"Setting key {settingKey} not found")


@app.put("/settings/{settingKey:path}")
def put_settings(settingKey: str, req: dict[str, Any]):
    try:
        value = req["newValue"]
        settings.set(settingKey, value)
        print(f"put_settings: {{'{settingKey}': {value}}}")
        return {}
    except KeyError as e:
        raise HTTPException(status_code=404, detail=str(e))


@app.put("/settings/restoreDefaults")
def restore_defaults():
    settings.restore_defaults()
    resp = {}
    print(f"restore_defaults: {resp}")
    return resp


@app.put("/cameraApp/init")
def cameraApp_init():
    cameraApp.gst_init()
    resp = {}
    print(f"init: {resp}")
    return resp


@app.put("/cameraApp/deinit")
def cameraApp_deinit():
    cameraApp.gst_deinit()
    resp = {}
    print(f"deinit: {resp}")
    return resp


@app.put("/cameraApp/preview-index")
def set_preview_index(req: dict[str, Any]):
    index = req["index"]
    active_index = cameraApp.setPreviewIndex(index)
    resp = {"index": active_index}
    print(f"set_preview_index: {resp}")
    return resp


@app.put("/cameraApp/capture")
def capture():
    cameraApp.capture()
    resp = {}
    print(f"capture: {resp}")
    return resp


@app.put("/cameraApp/recordStart")
def record_start():
    cameraApp.recordStart()
    resp = {}
    print(f"record_start: {resp}")
    return resp


@app.put("/cameraApp/recordStop")
def record_stop():
    cameraApp.recordStop()
    resp = {}
    print(f"record_stop: {resp}")
    return resp


@app.get("/cameraApp/settings")
def get_camera_settings():
    return cameraApp.get_settings()


@app.put("/cameraApp/settings")
def put_camera_settings(req: dict[str, Any]):
    new_settings = req.get("settings", req)
    return cameraApp.save_settings(new_settings)


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
def start_mapping():
    """Start the mapping process"""
    try:
        return mappingApp.start()
    except RuntimeError as e:
        raise HTTPException(status_code=409, detail=str(e))


@app.get("/mappingApp/initStatus")
def get_mapping_init_status():
    """Get mapping initialization status"""
    return mappingApp.get_init_status()


@app.put("/mappingApp/stop")
def stop_mapping():
    """Stop the mapping process and finalize results"""
    try:
        return mappingApp.stop()
    except RuntimeError as e:
        raise HTTPException(status_code=409, detail=str(e))


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
