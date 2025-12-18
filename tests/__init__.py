"""
DimenvuePro Server Integration & EE Tests

Test modules:
- test_camera_api.py: Camera initialization, capture, recording, settings
- test_mapping_api.py: Mapping lifecycle, state machine, settings
- test_catalog_api.py: Catalog CRUD operations, metadata, export
- test_storage_api.py: Internal/external storage management
- test_export_api.py: Export queue management, progress tracking
- test_ee.py: End-to-end workflow tests
- test_performance.py: Load and performance tests

Usage:
    # Run all tests
    pytest

    # Run specific test file
    pytest tests/test_camera_api.py

    # Run with markers
    pytest -m "integration"
    pytest -m "ee"
    pytest -m "not slow"
    pytest -m "not ros2"

    # Run with coverage
    pytest --cov=. --cov-report=html
"""
