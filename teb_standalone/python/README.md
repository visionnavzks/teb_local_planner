# Python Demo

This directory contains interactive examples for the standalone `pyteb` bindings.

## Available demos

- `interactive_demo.py`: desktop `matplotlib` viewer with a randomize button
- `web_demo.py`: browser-based dashboard with a left parameter panel, a central Plotly workspace view, and a right-side data panel

## What the demos do

- Creates a standalone TEB planner configuration
- Adds several static circular obstacles
- Plans a trajectory between a start pose and a goal pose
- Draws the obstacles, start/goal headings, and optimized trajectory
- Lets you resample the start and goal and replan immediately
- For the web demo, exposes the key planner parameters in the UI and shows trajectory metrics and samples next to the Plotly figure

## Build the Python module

From the repository root:

```bash
cmake -S teb_standalone -B teb_standalone/build
cmake --build teb_standalone/build --target pyteb
```

If CMake cannot find `pybind11`, install it first in an environment visible to CMake.
You also need the native standalone dependencies available to CMake, especially `SuiteSparse`, `g2o`, `Boost`, and `Eigen3`.

## Install the runtime dependencies

Recommended for macOS/Homebrew Python:

```bash
python3 -m venv teb_standalone/.venv
source teb_standalone/.venv/bin/activate
python -m pip install -r teb_standalone/python/requirements.txt
```

If you already have an activated virtual environment, installing the requirements is enough:

```bash
python -m pip install -r teb_standalone/python/requirements.txt
```

This installs the dependencies for both demos, including Dash and Plotly for the web UI.

## Run the desktop demo

From the repository root:

```bash
source teb_standalone/.venv/bin/activate
python teb_standalone/python/interactive_demo.py
```

The script automatically adds the default build directory at `teb_standalone/build` to `sys.path`.

On click of the `Random Start/Goal` button, the demo samples a new start pose and goal pose, replans, and refreshes the plot.

## Run the web demo

From the repository root:

```bash
source teb_standalone/.venv/bin/activate
python teb_standalone/python/web_demo.py
```

By default the server starts at `http://127.0.0.1:8050`.
You can override the bind address with environment variables:

```bash
source teb_standalone/.venv/bin/activate
TEB_WEB_HOST=0.0.0.0 TEB_WEB_PORT=8050 python teb_standalone/python/web_demo.py
```

The layout is optimized for desktop first, but it collapses into a single-column view on narrower screens.