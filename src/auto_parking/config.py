from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = PROJECT_ROOT / "config.yaml"
LOCAL_CONFIG = PROJECT_ROOT / "config.local.yaml"


def _deep_update(base, override):
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_update(base[key], value)
        else:
            base[key] = value
    return base


def load_config(path=None):
    config_path = Path(path) if path else DEFAULT_CONFIG
    with config_path.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    if path is None and LOCAL_CONFIG.exists():
        with LOCAL_CONFIG.open("r", encoding="utf-8") as f:
            local_config = yaml.safe_load(f) or {}
        config = _deep_update(config, local_config)

    return config
