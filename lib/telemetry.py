"""JSONL flight logger + frame stash. Context-manager safe."""

from __future__ import annotations

import datetime as _dt
import json
import os
import time
from pathlib import Path
from typing import Any, Optional, TextIO


def _iso_now() -> str:
    return _dt.datetime.now().astimezone().isoformat()


class FlightLog:
    """Writes ~/captures/YYYY-MM-DD/tello-<run_id>/kitchen.jsonl + frame jpegs."""

    def __init__(self, run_id: Optional[str] = None, root: Optional[Path | str] = None) -> None:
        self.run_id = run_id or time.strftime("%Y%m%d-%H%M%S")
        day = time.strftime("%Y-%m-%d")
        base = Path(root) if root is not None else Path.home() / "captures"
        self.run_dir: Path = base / day / f"tello-{self.run_id}"
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.jsonl_path: Path = self.run_dir / "kitchen.jsonl"
        self._fh: TextIO = self.jsonl_path.open("a", buffering=1)
        self._closed = False
        self.event("init", "run_start", run_id=self.run_id, run_dir=str(self.run_dir))

    def event(self, phase: str, kind: str, **fields: Any) -> None:
        if self._closed:
            return
        rec: dict[str, Any] = {"t": _iso_now(), "phase": phase, "kind": kind}
        rec.update(fields)
        self._fh.write(json.dumps(rec, default=str) + "\n")
        self._fh.flush()

    def frame(self, phase: str, cycle: int, frame_bgr: Any) -> str:
        """Write frame as phase-cycle-NN.jpg, return path. cv2 imported lazily."""
        import cv2  # lazy: keep tests cv2-free
        fname = f"{phase}-cycle-{cycle:02d}.jpg"
        path = self.run_dir / fname
        cv2.imwrite(str(path), frame_bgr)
        self.event(phase, "frame", cycle=cycle, frame_path=str(path))
        return str(path)

    def close(self) -> None:
        if self._closed:
            return
        self.event("exit", "run_end")
        try:
            self._fh.close()
        finally:
            self._closed = True

    def __enter__(self) -> "FlightLog":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if exc is not None:
            try:
                self.event("exit", "exception", exc_type=str(exc_type), exc=str(exc))
            except Exception:
                pass
        self.close()
