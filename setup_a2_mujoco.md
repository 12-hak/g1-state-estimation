# Setting Up Unitree A2 for MuJoCo Simulation

## Current Status

The Unitree A2 is a very new robot (announced late 2025, deliveries starting early 2026). Based on research:

- ✅ **SDK Support**: The `unitree_sdk2` has full A2 support including flip commands
- ❌ **URDF/MuJoCo Model**: Not yet publicly available in official repositories
- 🔍 **Available Models**: Go2, Go2W, B2, H1, H1-2, G1 (but no A2)

## Options to Get A2 URDF

### Option 1: Contact Unitree Directly (Recommended)
Since the A2 is brand new and you have SDK support, contact Unitree:
- Email: support@unitree.com or dev@unitree.com
- Request: A2 URDF/MJCF model for simulation
- Mention: You have SDK2 and want to test flip functionality in MuJoCo

### Option 2: Check Official Repositories
Monitor these repositories for A2 model updates:
- **Hugging Face**: https://huggingface.co/datasets/unitreerobotics/unitree_model
- **GitHub**: https://github.com/unitreerobotics/unitree_ros
- **SDK2 Updates**: Check if newer SDK2 versions include A2 models

### Option 3: Use B2 as Temporary Proxy
The B2 is also a quadruped and may have similar kinematics:
- B2 model is available in your `unitree_mujoco` workspace
- Test flip simulation concepts with B2
- Adapt to A2 when model becomes available

### Option 4: Create A2 Model from CAD (Advanced)
If you have access to A2 CAD files:
1. Export as URDF from CAD software
2. Convert URDF to MuJoCo MJCF using: `python -m mujoco.mjcf.urdf_to_mjcf`
3. Add collision meshes and tune parameters

## A2 Flip Commands (from SDK2)

The A2 supports these flip modes:
```cpp
const int ID_FRONT_FLIP = 10;  // Front flip
const int ID_BACK_FLIP = 11;   // Back flip
```

FSM States include:
- `FRONT_FLIP` (mode 10)
- `BACK_FLIP` (mode 11)

## Next Steps

1. **Immediate**: Try Option 1 - contact Unitree for A2 URDF
2. **Short-term**: Use B2 model to prototype flip simulation
3. **When A2 URDF available**: Follow conversion guide below

## Conversion Guide (When URDF Available)

### Step 1: Download A2 URDF
```bash
# Clone from Hugging Face (when available)
git clone https://huggingface.co/datasets/unitreerobotics/unitree_model
# Or download specific A2 folder
```

### Step 2: Convert to MuJoCo Format
```bash
# Install MuJoCo Python bindings
pip install mujoco

# Convert URDF to MJCF
python scripts/convert_a2_urdf_to_mjcf.py
```

### Step 3: Add to unitree_mujoco
```bash
# Copy converted files
cp -r A2_model/ unitree_mujoco/unitree_robots/a2/
```

### Step 4: Update Config
Edit `unitree_mujoco/simulate_python/config.py`:
```python
ROBOT = "a2"  # Add a2 to supported robots
```

### Step 5: Test Simulation
```bash
cd unitree_mujoco/simulate_python
python unitree_mujoco.py
```

## Testing Flip Commands

Once A2 model is available, create a flip test script similar to the GO2 example.

See: `create_a2_flip_demo.py` (to be created when model available)

## Resources

- **Unitree SDK2 A2 Examples**: `unitree_sdk2/example/a2/`
- **A2 Sport Client**: `unitree_sdk2/include/unitree/robot/a2/sport/sport_client.hpp`
- **A2 State Example**: `unitree_sdk2/example/a2/a2_sport_state.cpp`
- **Existing MuJoCo Models**: `unitree_mujoco/unitree_robots/`

## Alternative: Demo with GO2 Flips

Would you like me to create a working flip simulation demo using GO2 to demonstrate the concept?
This would show:
- How to trigger flip commands through SDK2
- How MuJoCo simulates dynamic movements
- Code structure you can adapt to A2 later
