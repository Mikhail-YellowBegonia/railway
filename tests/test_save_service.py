"""A0.1 save management regression."""
from pathlib import Path
import json
import tempfile

from model.save_service import SaveError, SaveService, migrate_document


root = Path(tempfile.mkdtemp(prefix="railway-save-service-"))
service = SaveService(root)

# Named save resolution and old-document migration.
assert service.path_for("yard") == root / "yard.geojson"
assert migrate_document({"type": "FeatureCollection"})["format_version"] == 1
assert service.list_saves() == []

empty = service.new_save("empty")
assert service.open_save("empty")["features"] == []
try:
    service.new_save("empty")
except SaveError:
    pass
else:
    raise AssertionError("new_save should not overwrite an existing slot")

target = service.save_as("yard", lambda p: p.write_text(
    json.dumps({"type": "FeatureCollection", "features": []}), encoding="utf-8"
))
assert target.exists()
assert service.open_document("yard")["format_version"] == 1
assert service.list_saves() == [empty, target]

# A failed writer must not replace the previous save.
before = target.read_text(encoding="utf-8")
try:
    service.atomic_write("yard", lambda _p: (_ for _ in ()).throw(ValueError("boom")))
except SaveError:
    pass
else:
    raise AssertionError("failed save should raise SaveError")
assert target.read_text(encoding="utf-8") == before

try:
    service.open_document("missing")
except SaveError:
    pass
else:
    raise AssertionError("missing save should raise SaveError")

print("✅ 存档服务：命名/版本迁移/原子写入/失败保留旧档通过")
