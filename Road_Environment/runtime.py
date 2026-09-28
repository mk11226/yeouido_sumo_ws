"""시스템 설치와 가상환경에서 SUMO 실행 경로 설정."""
import os
from pathlib import Path
import shutil
import sys

from Road_Environment.paths import ROOT


def prepare_runtime():
    # Debian/Ubuntu SUMO packages keep Python bindings outside site-packages.
    for directory in (Path(os.environ.get("SUMO_HOME", "/usr/share/sumo")) / "tools",):
        if directory.is_dir() and str(directory) not in sys.path:
            sys.path.append(str(directory))
    candidates = [Path(sys.executable).parent, ROOT / ".venv-skku/bin"]
    os.environ["PATH"] = os.pathsep.join(map(str, candidates)) + os.pathsep + os.environ.get("PATH", "")
    if shutil.which("sumo") is None:
        raise RuntimeError("SUMO is unavailable; use .venv-skku/bin/python or put SUMO on PATH")
