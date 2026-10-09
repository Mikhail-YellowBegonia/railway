"""Named, atomic save management for the GeoJSON session format.

The simulation still stores a session in one GeoJSON document.  This module
keeps file naming, compatibility/migration, and write safety out of the game
loop and out of the geometry serializer.
"""
from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path
from typing import Callable

CURRENT_FORMAT_VERSION = 1


class SaveError(RuntimeError):
    """Raised when a save cannot be opened or written safely."""


def migrate_document(document: dict) -> dict:
    """Return a current-format document without mutating the input.

    Existing GeoJSON saves predate an explicit version field.  They are
    intentionally treated as version 1, so adding version metadata is a
    backwards-compatible migration rather than a format fork.
    """
    if not isinstance(document, dict):
        raise SaveError("save document must be a JSON object")
    migrated = dict(document)
    version = migrated.get("format_version", 1)
    if not isinstance(version, int) or version < 1 or version > CURRENT_FORMAT_VERSION:
        raise SaveError(f"unsupported save format version: {version!r}")
    migrated["format_version"] = CURRENT_FORMAT_VERSION
    return migrated


class SaveService:
    """Resolve named saves and perform atomic document writes.

    ``directory`` is deliberately explicit; callers can use a project-local
    save directory in the game and a temporary directory in tests.
    """

    def __init__(self, directory: str | Path = ".") -> None:
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)

    @staticmethod
    def _filename(name: str | Path) -> str:
        raw = Path(name).name
        if raw in {"", ".", ".."} or raw != str(name) and Path(name).parent != Path("."):
            raise SaveError("save name must be a plain file name")
        if not raw.endswith(".geojson"):
            raw += ".geojson"
        if raw in {".geojson", "..geojson"}:
            raise SaveError("invalid save name")
        return raw

    def path_for(self, name: str | Path) -> Path:
        return self.directory / self._filename(name)

    def list_saves(self) -> list[Path]:
        return sorted(
            p for p in self.directory.glob("*.geojson")
            if not p.name.startswith(".") and not p.name.endswith(".tmp.geojson")
        )

    def open_document(self, name: str | Path) -> dict:
        path = self.path_for(name)
        try:
            with path.open("r", encoding="utf-8") as handle:
                return migrate_document(json.load(handle))
        except FileNotFoundError as exc:
            raise SaveError(f"save does not exist: {path.name}") from exc
        except (json.JSONDecodeError, OSError) as exc:
            raise SaveError(f"cannot open save {path.name}: {exc}") from exc

    def open_save(self, name: str | Path) -> dict:
        """Named-save open flow (alias kept explicit for callers/UI code)."""
        return self.open_document(name)

    def new_save(self, name: str | Path) -> Path:
        """Create an empty, valid GeoJSON save without overwriting an existing one."""
        target = self.path_for(name)
        if target.exists():
            raise SaveError(f"save already exists: {target.name}")
        return self.atomic_write(
            name,
            lambda path: path.write_text(
                json.dumps({
                    "type": "FeatureCollection",
                    "features": [],
                }),
                encoding="utf-8",
            ),
        )

    def atomic_write(self, name: str | Path, writer: Callable[[Path], None]) -> Path:
        """Write through a sibling temporary file and replace the target.

        The old save remains untouched if serialization, validation, or the
        final replace fails.
        """
        target = self.path_for(name)
        fd, tmp_name = tempfile.mkstemp(
            prefix=f".{target.stem}-", suffix=".tmp.geojson", dir=self.directory
        )
        os.close(fd)
        tmp = Path(tmp_name)
        try:
            writer(tmp)
            with tmp.open("r", encoding="utf-8") as handle:
                document = migrate_document(json.load(handle))
            with tmp.open("w", encoding="utf-8") as handle:
                json.dump(document, handle, indent=2)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(tmp, target)
            return target
        except Exception as exc:
            try:
                tmp.unlink(missing_ok=True)
            except OSError:
                pass
            if isinstance(exc, SaveError):
                raise
            raise SaveError(f"failed to write save {target.name}: {exc}") from exc

    def save_as(self, name: str | Path, writer: Callable[[Path], None]) -> Path:
        return self.atomic_write(name, writer)
