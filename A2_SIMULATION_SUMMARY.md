# Unitree A2 MuJoCo Simulation - Investigation Summary

## Investigation Results

### ✅ What We Found

1. **SDK2 Support for A2**
   - Location: `unitree_sdk2/example/a2/` and `unitree_sdk2/include/unitree/robot/a2/`
   - Files:
     - `a2_sport_client.cpp` - Sport mode control example
     - `a2_sport_state.cpp` - State monitoring example
     - `sport_client.hpp` - Sport API definitions
   
2. **A2 Flip Capabilities**
   - **Front Flip**: FSM State ID = 10 (`ID_FRONT_FLIP`)
   - **Back Flip**: FSM State ID = 11 (`ID_BACK_FLIP`)
   - These are built-in sport mode functions in the A2

3. **Available MuJoCo Models**
   - Go2, Go2W
   - B2, B2W (quadrupeds similar to A2)
   - H1, H1-2 (humanoids)
   - G1 (humanoid)

### ❌ What's Missing

1. **A2 URDF/MJCF Model**
   - Not available on Hugging Face (unitreerobotics/unitree_model)
   - Not available on GitHub (unitreerobotics/unitree_ros)
   - Reason: A2 is very new (late 2025/early 2026 release)

## Created Tools

### 1. `check_a2_urdf.py`
**Purpose**: Check for A2 URDF availability across official repositories

**Usage**:
```bash
python check_a2_urdf.py
```

**Features**:
- Checks Hugging Face datasets
- Checks GitHub repositories
- Verifies local SDK2 support
- Provides actionable recommendations

### 2. `convert_a2_to_mujoco.py`
**Purpose**: Convert A2 URDF to MuJoCo MJCF format (when available)

**Usage**:
```bash
# When A2 URDF becomes available:
python convert_a2_to_mujoco.py
```

**Features**:
- Automatic URDF to MJCF conversion
- Sets up proper directory structure
- Creates scene template
- Ready to use when URDF is released

### 3. `demo_b2_flip_simulation.py`
**Purpose**: Demonstrate flip simulation using B2 as A2 proxy

**Usage**:
```bash
# Terminal 1: Start MuJoCo simulation
cd unitree_mujoco/simulate_python
# Edit config.py: ROBOT = "b2"
python unitree_mujoco.py

# Terminal 2: Run flip demo
python demo_b2_flip_simulation.py
```

**Features**:
- Low-level flip control sequence
- Shows crouch → jump → rotate → land phases
- Includes notes for A2 Sport Mode API usage

### 4. `setup_a2_mujoco.md`
**Purpose**: Comprehensive guide for A2 setup

**Contents**:
- Current status
- Options to obtain URDF
- Conversion guide
- Testing procedures

## Recommended Next Steps

### Immediate Actions

1. **Contact Unitree for A2 URDF**
   ```
   Email: support@unitree.com or dev@unitree.com
   Subject: Request for A2 URDF/MJCF Model for MuJoCo Simulation
   
   Message:
   "I am developing with the Unitree A2 robot and have the SDK2 installed.
   I would like to test flip functionality (front flip, back flip) in MuJoCo
   simulation before deploying to hardware. Could you provide the A2 URDF
   or MJCF model files?"
   ```

2. **Monitor Official Repositories**
   - Hugging Face: https://huggingface.co/datasets/unitreerobotics/unitree_model
   - GitHub: https://github.com/unitreerobotics/unitree_ros
   - Check weekly for updates

### Short-term: Prototype with B2

While waiting for A2 URDF, use B2 to prototype:

1. **Test Flip Simulation Concepts**
   ```bash
   cd unitree_mujoco/simulate_python
   # Edit config.py: ROBOT = "b2"
   python unitree_mujoco.py
   ```

2. **Develop Control Logic**
   - Test flip sequences
   - Tune parameters
   - Validate physics
   - Code will transfer to A2

3. **Understand Sport Mode API**
   - Study `unitree_sdk2/example/a2/a2_sport_client.cpp`
   - The A2 uses high-level sport mode commands
   - Flips are triggered via FSM state changes

### When A2 URDF Becomes Available

1. **Download and Convert**
   ```bash
   python check_a2_urdf.py  # Will detect when available
   python convert_a2_to_mujoco.py  # Convert to MuJoCo format
   ```

2. **Integrate into unitree_mujoco**
   ```bash
   # Files will be in: unitree_mujoco/unitree_robots/a2/
   cd unitree_mujoco/simulate_python
   # Edit config.py: ROBOT = "a2"
   python unitree_mujoco.py
   ```

3. **Test Flip Commands**
   - Use Sport Mode API (not low-level control)
   - Trigger via FSM state: `ID_FRONT_FLIP` (10) or `ID_BACK_FLIP` (11)

## A2 Flip Implementation (Future)

When A2 model is available, flip control will be simpler than B2 demo:

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.a2.sport import SportClient

# Initialize
ChannelFactoryInitialize(1, "lo")
client = SportClient()
client.Init()

# Execute front flip
# Method 1: If direct API exists
client.FrontFlip()

# Method 2: Via FSM state switch
client.SwitchMode(10)  # ID_FRONT_FLIP

# Execute back flip
client.SwitchMode(11)  # ID_BACK_FLIP
```

The robot's built-in controller handles all the complex dynamics!

## Key Differences: A2 vs B2

| Feature | B2 | A2 |
|---------|----|----|
| Control | Low-level joint control | Sport Mode API |
| Flip Implementation | Manual sequence programming | Built-in FSM states |
| Complexity | High (must program all phases) | Low (single API call) |
| URDF Available | ✅ Yes | ❌ Not yet |
| MuJoCo Model | ✅ Yes | ❌ Not yet |

## Resources

- **SDK2 A2 Examples**: `unitree_sdk2/example/a2/`
- **A2 Sport API**: `unitree_sdk2/include/unitree/robot/a2/sport/sport_client.hpp`
- **MuJoCo Models**: `unitree_mujoco/unitree_robots/`
- **Hugging Face**: https://huggingface.co/datasets/unitreerobotics/unitree_model
- **GitHub**: https://github.com/unitreerobotics/unitree_ros

## Questions?

If you need help with:
- Setting up B2 simulation for prototyping
- Understanding the Sport Mode API
- Converting URDF when available
- Implementing flip sequences

Just ask!
