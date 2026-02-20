#!/usr/bin/env python3
"""
URDF to MuJoCo MJCF Converter for Unitree A2
This script will convert A2 URDF files to MuJoCo MJCF format when available.
"""

import os
import sys
from pathlib import Path

def convert_urdf_to_mjcf(urdf_path, output_dir):
    """
    Convert URDF to MuJoCo MJCF format
    
    Args:
        urdf_path: Path to the URDF file
        output_dir: Directory to save the converted MJCF file
    """
    try:
        import mujoco
        print(f"[*] Converting {urdf_path} to MJCF...")
        
        # Load URDF
        model = mujoco.MjModel.from_xml_path(str(urdf_path))
        
        # Save as MJCF
        output_path = Path(output_dir) / "a2.xml"
        mujoco.mj_saveLastXML(str(output_path), model)
        
        print(f"[+] Conversion complete!")
        print(f"    Output: {output_path}")
        return True
        
    except ImportError:
        print("[-] MuJoCo Python bindings not installed")
        print("    Install with: pip install mujoco")
        return False
    except Exception as e:
        print(f"[-] Conversion failed: {e}")
        return False

def setup_a2_mujoco_directory(source_dir, target_dir="unitree_mujoco/unitree_robots/a2"):
    """
    Set up A2 directory structure for MuJoCo simulation
    
    Args:
        source_dir: Directory containing A2 URDF and meshes
        target_dir: Target directory in unitree_mujoco
    """
    target_path = Path(target_dir)
    target_path.mkdir(parents=True, exist_ok=True)
    
    print(f"[*] Setting up A2 MuJoCo directory at {target_path}")
    
    # Copy meshes
    source_meshes = Path(source_dir) / "meshes"
    if source_meshes.exists():
        import shutil
        target_meshes = target_path / "meshes"
        if target_meshes.exists():
            shutil.rmtree(target_meshes)
        shutil.copytree(source_meshes, target_meshes)
        print(f"[+] Copied meshes to {target_meshes}")
    
    # Create scene.xml template
    scene_xml = target_path / "scene.xml"
    scene_content = """<mujoco model="a2_scene">
  <compiler angle="radian" meshdir="meshes" autolimits="true"/>
  
  <option timestep="0.002" iterations="50" solver="Newton" gravity="0 0 -9.81"/>
  
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1=".1 .2 .3" rgb2=".2 .3 .4"/>
    <material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance=".2"/>
  </asset>
  
  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="grid" condim="3"/>
    
    <include file="a2.xml"/>
  </worldbody>
  
  <actuator>
    <!-- Joint actuators will be auto-generated from URDF conversion -->
  </actuator>
</mujoco>
"""
    
    with open(scene_xml, 'w') as f:
        f.write(scene_content)
    print(f"[+] Created scene template: {scene_xml}")
    
    return target_path

def main():
    print("="*60)
    print("Unitree A2 URDF to MuJoCo Converter")
    print("="*60)
    
    # Check if A2 URDF is available
    possible_locations = [
        "unitree_models_download/unitree_model/A2",
        "A2",
        "unitree_ros/robots/a2_description",
    ]
    
    urdf_file = None
    for loc in possible_locations:
        potential_urdf = Path(loc) / "urdf" / "a2.urdf"
        if potential_urdf.exists():
            urdf_file = potential_urdf
            print(f"[+] Found A2 URDF at: {urdf_file}")
            break
    
    if not urdf_file:
        print("[-] A2 URDF not found in expected locations")
        print("    Searched:")
        for loc in possible_locations:
            print(f"      - {loc}/urdf/a2.urdf")
        print("\n    Please:")
        print("    1. Download A2 URDF from Unitree")
        print("    2. Place in one of the above locations")
        print("    3. Run this script again")
        return
    
    # Convert URDF to MJCF
    output_dir = "a2_mjcf_output"
    Path(output_dir).mkdir(exist_ok=True)
    
    success = convert_urdf_to_mjcf(urdf_file, output_dir)
    
    if success:
        # Set up MuJoCo directory structure
        source_dir = urdf_file.parent.parent
        setup_a2_mujoco_directory(source_dir)
        
        print("\n" + "="*60)
        print("NEXT STEPS")
        print("="*60)
        print("1. Review the converted MJCF file")
        print("2. Adjust collision meshes if needed")
        print("3. Add actuator definitions")
        print("4. Update unitree_mujoco/simulate_python/config.py:")
        print("   ROBOT = 'a2'")
        print("5. Test simulation:")
        print("   cd unitree_mujoco/simulate_python")
        print("   python unitree_mujoco.py")

if __name__ == "__main__":
    main()
