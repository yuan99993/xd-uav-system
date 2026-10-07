"""Validated model registry used by the shared detector and web API."""

from pathlib import Path
import hashlib
import re

import yaml


_DIGEST = re.compile(r"^[0-9a-f]{64}$")


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_model_registry(path, package_root):
    registry_path = Path(path).expanduser().resolve()
    root = Path(package_root).resolve()
    with registry_path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}
    if raw.get("schema_version") != 1 or not isinstance(raw.get("models"), dict):
        raise ValueError("model registry requires schema_version 1 and models mapping")
    models = {}
    for model_id, entry in raw["models"].items():
        if not re.fullmatch(r"[a-z0-9_-]+", str(model_id)):
            raise ValueError("invalid model_id: %s" % model_id)
        item = dict(entry or {})
        pt_path = (root / str(item.get("pt_path", ""))).resolve()
        if root not in pt_path.parents or not pt_path.is_file():
            raise ValueError("registered model file unavailable: %s" % pt_path)
        digest = str(item.get("pt_sha256", "")).lower()
        if _DIGEST.fullmatch(digest) is None:
            raise ValueError("registered model has invalid SHA-256: %s" % model_id)
        if _sha256(pt_path) != digest:
            raise ValueError("registered model SHA-256 mismatch: %s" % model_id)
        classes = item.get("class_names", [])
        if not isinstance(classes, list) or not classes:
            raise ValueError("registered model needs class_names: %s" % model_id)
        item["model_id"] = str(model_id)
        item["pt_path"] = str(pt_path)
        item["pt_sha256"] = digest
        item["class_names"] = [str(value) for value in classes]
        item["input_size"] = int(item.get("input_size", 640))
        engine = str(item.get("engine_path", "") or "")
        engine_path = (root / engine).resolve() if engine else None
        if engine_path is not None and root not in engine_path.parents:
            raise ValueError("registered engine escapes package root: %s" % model_id)
        item["engine_path"] = str(engine_path) if engine_path else ""
        item["engine_prepared"] = bool(
            item["engine_path"] and Path(item["engine_path"]).is_file())
        item["engine_sha256"] = str(item.get("engine_sha256", "")).lower()
        engine_source = str(item.get("engine_source_model_sha256", "")).lower()
        if engine and engine_source != digest:
            raise ValueError(
                "registered engine source SHA-256 mismatch: %s" % model_id)
        item["engine_source_model_sha256"] = engine_source
        if item["engine_prepared"]:
            if (_DIGEST.fullmatch(item["engine_sha256"]) is None or
                    _sha256(item["engine_path"]) != item["engine_sha256"]):
                raise ValueError("registered engine SHA-256 mismatch: %s" % model_id)
        models[str(model_id)] = item
    return models


def identify_model(models, source_sha256):
    digest = str(source_sha256 or "").lower()
    matches = [model_id for model_id, item in models.items()
               if item["pt_sha256"] == digest]
    return matches[0] if len(matches) == 1 else "unknown"
