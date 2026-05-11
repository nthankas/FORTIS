"""Shared pytest fixtures for the urdf_cleanup test suite."""

from __future__ import annotations

from pathlib import Path

import pytest

FIXTURES = Path(__file__).resolve().parent / "fixtures"


@pytest.fixture
def minimal_export_path() -> Path:
    return FIXTURES / "minimal_export.urdf"


@pytest.fixture
def realistic_export_path() -> Path:
    return FIXTURES / "realistic_export.urdf"


@pytest.fixture
def expected_minimal_path() -> Path:
    return FIXTURES / "minimal_expected.urdf"
