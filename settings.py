import json
import threading
import shutil

# TODO: remove the singleton shit


class Settings:
    """
    A singleton class for managing settings.
    Get/set are done persistently to a JSON file.
    Settings are nested and can be get/set by key path, separated by '/'.
    """
    _instance = None
    _lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            with cls._lock:
                # Another thread could have created the instance
                # before this one acquired the lock.
                if cls._instance is None:
                    cls._instance = super(Settings, cls).__new__(cls)
        return cls._instance

    def __init__(self, json_path: str = None):
        # The __init__ is called every time `Settings()` is invoked,
        # so we need a flag to ensure initialization happens only once.
        if not hasattr(self, '_initialized'):
            with self._lock:
                if not hasattr(self, '_initialized'):
                    self._path = str(json_path)
                    self._load_settings()
                    self._initialized = True

    def _load_settings(self):
        if not self._path:
            print("Settings path not provided. Starting with empty settings.")
            return
        try:
            default_path = self._path.replace(
                "settings.json", "settings_default.json")
            with open(self._path, "r") as f:
                data = json.load(f)
            with open(default_path, "r") as f:
                default_data = json.load(f)
            self._data = data
            self._data['defaults'] = default_data
        except FileNotFoundError:
            print(f"settings.json not found. Starting with empty settings.")
            data = {}
        except json.JSONDecodeError:
            print(
                f"Error decoding {self._path}. Starting with empty settings.")
            data = {}

        return data

    def _save_settings(self):
        default_data = self._data.pop('defaults')
        with open(self._path, "w") as f:
            json.dump(self._data, f, indent=4)
        self._data['defaults'] = default_data

    def _key_exists(self, keys: str):
        parts = keys.split("/")
        d = self._data
        for part in parts:
            if part not in d:
                return False
            d = d[part]
        return True

    def _get_value(self, keys: str):
        parts = keys.split("/")
        d = self._data
        for part in parts:
            d = d[part]
        return d

    def _set_value(self, keys: str, value: any):
        parts = keys.split("/")
        d = self._data
        for part in parts[:-1]:
            d = d[part]  # This will raise KeyError if a path part is missing

        if not isinstance(d, dict):
            raise TypeError(
                f"Cannot set key on a non-dictionary element: '{'.'.join(parts[:-1])}'")

        d[parts[-1]] = value

    def get(self, key: str):
        if not self._key_exists(key):
            raise KeyError(f"Setting key '{key}' not found")
        return self._get_value(key)

    def set(self, key: str, value: any):
        if not self._key_exists(key):
            raise KeyError(f"Setting key '{key}' not found")
        self._set_value(key, value)
        self._save_settings()

    def restore_defaults(self):
        default_data = self._data['default']
        self._data = default_data
        self._save_settings()

    @property
    def data(self):
        return self._data
