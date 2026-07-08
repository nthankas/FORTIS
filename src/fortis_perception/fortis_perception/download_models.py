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

#: Pinned model source: standard ultralytics YOLOv8n COCO export, pinned to a
#: HuggingFace revision hash so the content can never change under the URL.
MODEL_URL = ("https://huggingface.co/SpotLab/YOLOv8Detection/resolve/"
             "3005c6751fb19cdeb6b10c066185908faf66a097/yolov8n.onnx")
MODEL_FILENAME = "yolov8n.onnx"

#: Verified 2026-07-08 against the revision-pinned URL above; enforced on
#: every download. Inference requires OpenCV >= 4.7 (see detectors.py).
EXPECTED_SHA256 = "dd48a79dd7fec8ca25fde4eca742ff7bca23b27e2e903eb23bc1d9f83a459bd2"

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
