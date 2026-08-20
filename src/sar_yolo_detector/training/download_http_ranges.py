#!/usr/bin/env python3
"""Resumable parallel HTTP range downloader for large public datasets."""

import argparse
import hashlib
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import requests


def download_part(url: str, output: Path, start: int, end: int, retries: int) -> None:
    expected = end - start + 1
    for attempt in range(retries):
        existing = output.stat().st_size if output.is_file() else 0
        if existing > expected:
            output.unlink()
            existing = 0
        if existing == expected:
            return
        request_start = start + existing
        headers = {"Range": f"bytes={request_start}-{end}"}
        try:
            with requests.get(url, headers=headers, stream=True, timeout=(30, 120)) as response:
                if response.status_code not in (200, 206):
                    raise RuntimeError(f"HTTP {response.status_code}")
                # A proxy may ignore Range. Never append a complete response to a partial part.
                if existing and response.status_code == 200:
                    output.unlink()
                    continue
                mode = "ab" if existing else "wb"
                with output.open(mode) as stream:
                    for block in response.iter_content(1024 * 1024):
                        if block:
                            stream.write(block)
            if output.stat().st_size == expected:
                return
        except Exception:
            if attempt + 1 == retries:
                raise

    raise RuntimeError(f"failed to download {output}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--size", type=int, required=True)
    parser.add_argument("--chunk", type=int, default=16 * 1024 * 1024)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--sha256", default="")
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    parts = args.output.parent / (args.output.name + ".parts")
    parts.mkdir(exist_ok=True)
    ranges = [(start, min(args.size - 1, start + args.chunk - 1))
              for start in range(0, args.size, args.chunk)]
    with ThreadPoolExecutor(max_workers=args.workers) as executor:
        jobs = [executor.submit(download_part, args.url, parts / f"part_{index:04d}",
                                 start, end, 10)
                for index, (start, end) in enumerate(ranges)]
        for job in as_completed(jobs):
            job.result()
    with args.output.open("wb") as target:
        for index in range(len(ranges)):
            target.write((parts / f"part_{index:04d}").read_bytes())
    if args.output.stat().st_size != args.size:
        raise SystemExit("assembled file size does not match expected size")
    if args.sha256:
        digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
        if digest != args.sha256:
            raise SystemExit(f"sha256 mismatch: {digest}")
    print(f"downloaded {args.output} ({args.output.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
