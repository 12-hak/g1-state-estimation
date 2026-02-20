# Installing unitree_sdk2py for A2 Simulation

## Quick Install

The `unitree_sdk2py` Python package is required for Unitree robot simulation and control.

### Option 1: Install from GitHub (Recommended)

```bash
# Clone the Python SDK repository
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git

# Navigate to the directory
cd unitree_sdk2_python

# Install in editable mode
pip install -e .
```

### Option 2: Install from PyPI (if available)

```bash
pip install unitree_sdk2py
```

## Verify Installation

After installation, verify it works:

```bash
python -c "import unitree_sdk2py; print('SDK2 Python installed successfully!')"
```

## For Windows Users

If you encounter issues on Windows, you may need:

1. **Install Visual C++ Build Tools**
   - Download from: https://visualstudio.microsoft.com/visual-cpp-build-tools/
   - Required for compiling C++ extensions

2. **Use WSL (Windows Subsystem for Linux)**
   ```bash
   # In WSL terminal
   sudo apt update
   sudo apt install python3-pip git
   git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
   cd unitree_sdk2_python
   pip3 install -e .
   ```

## Dependencies

The SDK will automatically install required dependencies:
- `numpy`
- `cyclonedds` (for DDS communication)
- Other Python bindings

## Next Steps

After installation:

1. **Test with existing examples**:
   ```bash
   cd unitree_mujoco/simulate_python
   python unitree_mujoco.py
   ```

2. **Run the A2 checker**:
   ```bash
   python check_a2_urdf.py
   ```

3. **Try the B2 flip demo** (when ready):
   ```bash
   python demo_b2_flip_simulation.py
   ```

## Troubleshooting

### Import Error: No module named 'cyclonedds'
```bash
pip install cyclonedds
```

### Permission Denied
```bash
pip install --user -e .
```

### Build Errors on Windows
Use WSL or install Visual C++ Build Tools

## Alternative: Use Existing MuJoCo Examples

The `unitree_mujoco` examples already use `unitree_sdk2py`. You can:

1. Check if it's already installed in your environment
2. Look at working examples in `unitree_mujoco/example/python/`
3. Use the bridge in `unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py`
