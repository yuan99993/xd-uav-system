"""Repository-owned terrain resource locations shared by both Web modes."""

from pathlib import Path


# This module lives at src/web/backend/app/.  The tracked, canonical terrain
# source lives at src/terrain-navigation/; src/web/terrain-navigation is an
# ignored handoff copy and must never be required at runtime.
SOURCE_ROOT = Path(__file__).resolve().parents[3]
CANONICAL_DAVOSDORF_DEM = (
    SOURCE_ROOT / "terrain-navigation" / "resources" / "davosdorf.tif")
