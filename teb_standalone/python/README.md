# Python Demo

This directory contains a small interactive example for the standalone `pyteb` bindings.

## What the demo does

- Creates a standalone TEB planner configuration
- Adds several static circular obstacles
- Plans a trajectory between a start pose and a goal pose
- Draws the obstacles, start/goal headings, and optimized trajectory with `matplotlib`
- Lets you click a button to randomly resample the start and goal and replan immediately

## Build the Python module

From the repository root:

```bash
cmake -S teb_standalone -B teb_standalone/build
cmake --build teb_standalone/build --target pyteb
```

If CMake cannot find `pybind11`, install it first in an environment visible to CMake.
You also need the native standalone dependencies available to CMake, especially `SuiteSparse`, `g2o`, `Boost`, and `Eigen3`.

## Install the runtime dependencies

```bash
python3 -m pip install numpy matplotlib
```

## Run the demo

From the repository root:

```bash
python3 teb_standalone/python/interactive_demo.py
```

The script automatically adds the default build directory at `teb_standalone/build` to `sys.path`.

On click of the `Random Start/Goal` button, the demo samples a new start pose and goal pose, replans, and refreshes the plot.