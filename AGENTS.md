# AGENTS.md - Context & Guidelines

## 1. Project Overview
This project converts 2D transmission tower drawings (TXT format) into 3D models (Excel/JSON output).
- **Core Logic**: Reconstructs 3D coordinates from dual-view (front/side) and single-view (front only) inputs.
- **Entry Point**: `testAll.py` (Orchestrates the full pipeline via `tran2dto3d.py`).
- **Key Modules**:
    - `tran2dto3d.py`: Main controller.
    - `tower_body_reconstruction.py`: Handles tower body (dual-view).
    - `dual_view_core.py`: Pure algorithmic core for 3D reconstruction.
    - `xintrans.py`: Handles "stretcher" (danjia) components.

## 2. Build & Test Commands

### Environment
- Python 3.9+ recommended.
- **Dependencies**: `pandas`, `matplotlib`, `openpyxl`.
  ```bash
  pip install pandas matplotlib openpyxl
  ```

### Running the Project
This project uses script-based execution rather than a test runner like pytest.

- **Run Full Pipeline**:
  ```bash
  python testAll.py
  ```
  *Note: This script calls `tran2dto3d.tran2dto3d` with hardcoded paths. Modify paths in `testAll.py` if needed.*

- **Run Visualization**:
  ```bash
  python draw_tower_3d.py
  ```

- **Unit Testing**:
  There are no formal unit tests (pytest/unittest). Testing is done by running `testAll.py` and checking the output files in the specified `savepath_ui` directory.

## 3. Code Style Guidelines

### Formatting
- **Indentation**: 4 spaces (Standard Python).
- **Line Length**: No strict limit, but keep readable (approx 88-100 chars).
- **Imports**:
    - Standard libraries first (`typing`, `math`, `os`).
    - Third-party (`pandas`, `matplotlib`) second.
    - Local imports last.
    - Use `from typing import ...` for type hints.

### Type Hinting (IMPORTANT)
- **Strongly Recommended**: Use Python's `typing` module.
- **Style**:
  ```python
  from typing import List, Dict, Tuple, TypeAlias

  Point3D: TypeAlias = Tuple[float, float, float]

  def calculate_distance(p1: Point3D, p2: Point3D) -> float:
      ...
  ```
- **Goal**: fully typed function signatures for core logic (like in `dual_view_core.py`).

### Naming Conventions
- **Functions & Variables**: `snake_case` (e.g., `calculate_height`, `node_list`).
- **Classes**: `PascalCase` (if classes are used, though functional style prevails here).
- **Constants**: `UPPER_CASE`.
- **Private Internal**: Prefix with `_` (e.g., `_collect_endpoints`).

### Documentation
- **Docstrings**: Required for complex functions in core modules.
  ```python
  def center_props(front3d: Model3DData, right3d: Model3DData):
      """
      Calculate the center coordinates and minimum Z value.
      Returns: ((x_center, y_center, 0), z_min)
      """
  ```
- **Inline Comments**: Use sparingly to explain *why*, not *what*.

## 4. Architecture & Patterns

### Data Structures
- **Nodes (Jiedian)**:
  - Type 11: Real 3D coordinates (X, Y, Z).
  - Type 12: Reference-based (RefX, RefY, RealZ) to preserve precision.
- **Members (Ganjian)**: Defined by connecting two Node IDs.

### Design Principles
- **Separation of Concerns**: `dual_view_core.py` should remain pure logic (no file I/O). I/O belongs in `io_utils.py` or processor scripts.
- **Coordinate Systems**:
    - 2D inputs: View-specific coordinates.
    - 3D internal: Unified coordinate system after reconstruction.
    - Output: Normalized IDs (1-500 range) for "SmartTower" compatibility.

### Common Pitfalls
- **ID String vs Int**: IDs are often strings (e.g., "508_1"). Be careful not to cast to int prematurely.
- **Floating Point**: Use `math.isclose` or epsilon comparisons for coordinate matching.

## 5. Agent Instructions
- **When Editing**: Prefer modifying `dual_view_core.py` for algorithmic changes and `tower_body_reconstruction.py` for process flow changes.
- **When Testing**: Always run `python testAll.py` to verify the end-to-end flow is not broken.
- **New Features**: If adding a new calculation, add it to `dual_view_core.py` with full type hints and unit-testable design.
