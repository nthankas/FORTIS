"""
Model fetcher for detection_node (stub).

Will download and cache the object-detection model weights that
detection_node loads at startup, verifying checksums so the perception
stack never silently runs a stale model.
"""


def main():
    """Entry point registered as the `download_models` console script."""
    print(
        "download_models (stub): will fetch and cache detection model "
        "weights for fortis_perception detection_node. Nothing to do yet."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
