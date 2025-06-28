from __future__ import annotations

import yaml
from pathlib import Path
from typing import Any, Dict


class UniversalConfig:
    """Lightweight configuration aggregator for the multi-platform stack."""

    def __init__(self, main_config_path: str | None = None):
        self.robot_configs: Dict[str, Any] = {}
        self.control_configs: Dict[str, Any] = {}
        self.task_configs: Dict[str, Any] = {}

        self._load_directory(Path('config/robots'), self.robot_configs)
        self._load_directory(Path('config/control'), self.control_configs, optional=True)
        self._load_directory(Path('config/tasks'), self.task_configs, optional=True)

        self._main_config: Dict[str, Any] = {}
        if main_config_path:
            with open(main_config_path, 'r') as f:
                self._main_config = yaml.safe_load(f)

    # ------------------------------------------------------------------

    def _load_directory(self, path: Path, target: Dict[str, Any], *, optional: bool = False):
        if not path.exists():
            if optional:
                return
            raise FileNotFoundError(path)
        for file in path.glob('*.yaml'):
            with open(file, 'r') as f:
                data = yaml.safe_load(f)
            key = file.stem  # filename without extension
            target[key] = data

    # ------------------------------------------------------------------

    def get(self, key: str, default=None):
        return self._main_config.get(key, default) 