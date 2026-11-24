#!/usr/bin/env python3
"""Register the package LetterboxResize transform, then run upstream D-FINE."""
import argparse
import runpy
import sys
from pathlib import Path


def main():
    package_root = Path(__file__).resolve().parents[2]
    default_repo = package_root / "third_party" / "D-FINE"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo", type=Path, default=default_repo)
    parser.add_argument("train_args", nargs=argparse.REMAINDER,
                        help="arguments passed to D-FINE train.py after --")
    args = parser.parse_args()
    forwarded = list(args.train_args)
    if forwarded and forwarded[0] == "--":
        forwarded.pop(0)
    if not forwarded or not any(value in ("-c", "--config") for value in forwarded):
        parser.error("pass D-FINE training arguments after --, including -c/--config")
    repo = args.repo.resolve()
    if not (repo / "train.py").is_file():
        parser.error(f"D-FINE train.py not found under {repo}")
    sys.path.insert(0, str(repo))
    # Importing the module registers it in D-FINE's GLOBAL_CONFIG workspace.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import letterbox_resize  # noqa: F401
    sys.argv = [str(repo / "train.py")] + forwarded
    runpy.run_path(str(repo / "train.py"), run_name="__main__")


if __name__ == "__main__":
    main()
