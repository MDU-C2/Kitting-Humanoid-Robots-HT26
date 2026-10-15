"""Lets the tests import karis_perception from the source tree too (without colcon)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
