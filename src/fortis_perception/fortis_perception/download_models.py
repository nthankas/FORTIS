"""
Download detection model weights into the local cache.

Console script (``download_models``). Fetches the pinned YOLOv8n ONNX
export with urllib only (no extra deps), verifies SHA256 when a pin is
set, and writes atomically (temp file + rename) so a partial download
never poisons the cache.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import sys
import tempfile
import urllib.request

#: Pinned model source (Unity's mirror of the ultralytics YOLOv8n ONNX export).
MODEL_URL = ("https://huggingface.co/unity/inference-engine-yolo/"
             "resolve/main/yolov8n.onnx")
MODEL_FILENAME = "yolov8n.onnx"

# TODO: pin after the first verified download. Run this script once on a
# trusted network, take the sha256 it prints, and paste it here. While None
# the hash is printed but not enforced.
EXPECTED_SHA256 = None

DEFAULT_DEST = os.path.join("~", ".cache", "fortis", "models")


def _sha256(path: str) -> str:
    """Return the hex SHA256 of a file, streaming in 1 MiB chunks."""
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main(argv=None) -> int:
    """Fetch the model into --dest and return a process exit code."""
    parser = argparse.ArgumentParser(
        description="Download the YOLOv8n ONNX weights for fortis_perception."
    )
    parser.add_argument("--dest", default=DEFAULT_DEST,
                        help="target directory (default: %(default)s)")
    parser.add_argument("--force", action="store_true",
                        help="re-download even if the file already exists")
    args = parser.parse_args(argv)

    dest_dir = os.path.expanduser(args.dest)
    os.makedirs(dest_dir, exist_ok=True)
    target = os.path.join(dest_dir, MODEL_FILENAME)

    if os.path.isfile(target) and not args.force:
        print(f"already present: {target} (sha256 {_sha256(target)})")
        return 0

    print(f"downloading {MODEL_URL}")
    fd, tmp_path = tempfile.mkstemp(dir=dest_dir, suffix=".part")
    try:
        with os.fdopen(fd, "wb") as tmp, urllib.request.urlopen(MODEL_URL) as resp:
            while True:
                chunk = resp.read(1 << 20)
                if not chunk:
                    break
                tmp.write(chunk)
        digest = _sha256(tmp_path)
        if EXPECTED_SHA256 is not None and digest != EXPECTED_SHA256:
            print(f"sha256 mismatch: expected {EXPECTED_SHA256}, got {digest}",
                  file=sys.stderr)
            return 1
        os.replace(tmp_path, target)
    except OSError as exc:  # urllib.error.URLError subclasses OSError
        print(f"download failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if os.path.exists(tmp_path):
            os.unlink(tmp_path)

    print(f"saved {target}")
    suffix = "" if EXPECTED_SHA256 else "  <- pin this as EXPECTED_SHA256"
    print(f"sha256 {digest}{suffix}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
