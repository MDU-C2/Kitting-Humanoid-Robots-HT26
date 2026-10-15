"""Code style (flake8, settings in setup.cfg)."""
import shutil
import subprocess
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parents[1]


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    if shutil.which("flake8") is None:
        pytest.skip("flake8 is not installed")
    r = subprocess.run(["flake8", "--config", str(PKG / "setup.cfg"), "karis_perception", "test"],
                       cwd=PKG, capture_output=True, text=True)
    assert r.returncode == 0, "flake8 found problems:\n" + r.stdout + r.stderr
