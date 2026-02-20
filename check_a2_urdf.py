#!/usr/bin/env python3
"""
Script to check for and download Unitree A2 URDF/model files.
This script checks official Unitree repositories for A2 model availability.
"""

import os
import sys
import subprocess
import requests
from pathlib import Path

def check_huggingface_a2():
    """Check if A2 model is available on Hugging Face"""
    print("[*] Checking Hugging Face for A2 model...")
    
    api_url = "https://huggingface.co/api/datasets/unitreerobotics/unitree_model/tree/main"
    
    try:
        response = requests.get(api_url, timeout=10)
        if response.status_code == 200:
            files = response.json()
            a2_files = [f for f in files if 'A2' in f.get('path', '').upper()]
            
            if a2_files:
                print("[+] Found A2 model on Hugging Face!")
                for f in a2_files:
                    print(f"   - {f['path']}")
                return True
            else:
                print("[-] A2 model not found on Hugging Face yet")
                available = [f['path'] for f in files if f['type'] == 'directory']
                print(f"   Available models: {', '.join(available)}")
                return False
    except Exception as e:
        print(f"[!] Error checking Hugging Face: {e}")
        return False

def check_github_unitree_ros():
    """Check GitHub unitree_ros for A2"""
    print("\n[*] Checking GitHub unitree_ros for A2...")
    
    api_url = "https://api.github.com/repos/unitreerobotics/unitree_ros/contents"
    
    try:
        response = requests.get(api_url, timeout=10)
        if response.status_code == 200:
            contents = response.json()
            a2_dirs = [item for item in contents if 'a2' in item['name'].lower()]
            
            if a2_dirs:
                print("[+] Found A2 related content on GitHub!")
                for item in a2_dirs:
                    print(f"   - {item['name']} ({item['type']})")
                return True
            else:
                print("[-] A2 not found in unitree_ros yet")
                dirs = [item['name'] for item in contents if item['type'] == 'dir']
                print(f"   Available: {', '.join(dirs[:10])}")
                return False
    except Exception as e:
        print(f"[!] Error checking GitHub: {e}")
        return False

def download_a2_from_huggingface():
    """Download A2 model from Hugging Face using git"""
    print("\n[*] Attempting to download A2 model from Hugging Face...")
    
    target_dir = Path("unitree_models_download")
    target_dir.mkdir(exist_ok=True)
    
    try:
        # Clone the repository
        cmd = [
            "git", "clone",
            "--depth", "1",
            "--filter=blob:none",
            "--sparse",
            "https://huggingface.co/datasets/unitreerobotics/unitree_model",
            str(target_dir / "unitree_model")
        ]
        
        print(f"Running: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)
        
        # Sparse checkout for A2 only
        os.chdir(target_dir / "unitree_model")
        subprocess.run(["git", "sparse-checkout", "init", "--cone"], check=True)
        subprocess.run(["git", "sparse-checkout", "set", "A2"], check=True)
        
        print("[+] Download complete!")
        print(f"   Location: {target_dir / 'unitree_model' / 'A2'}")
        return True
        
    except subprocess.CalledProcessError as e:
        print(f"[-] Download failed: {e}")
        print("   Note: A2 folder might not exist yet in the repository")
        return False
    except Exception as e:
        print(f"[-] Error: {e}")
        return False

def check_local_sdk2_a2():
    """Check if local SDK2 has A2 examples"""
    print("\n[*] Checking local unitree_sdk2 for A2 support...")
    
    sdk2_path = Path("unitree_sdk2")
    if not sdk2_path.exists():
        print("[-] unitree_sdk2 not found in current directory")
        return False
    
    a2_example = sdk2_path / "example" / "a2"
    a2_include = sdk2_path / "include" / "unitree" / "robot" / "a2"
    
    if a2_example.exists() and a2_include.exists():
        print("[+] SDK2 has A2 support!")
        print(f"   Examples: {a2_example}")
        print(f"   Headers: {a2_include}")
        
        # List A2 example files
        if a2_example.exists():
            examples = list(a2_example.glob("*.cpp"))
            print(f"   Example files: {', '.join([e.name for e in examples])}")
        
        return True
    else:
        print("[-] A2 support not found in local SDK2")
        return False

def suggest_next_steps(has_hf, has_gh, has_sdk2):
    """Suggest next steps based on what was found"""
    print("\n" + "="*60)
    print("RECOMMENDATIONS")
    print("="*60)
    
    if has_hf:
        print("\n[+] A2 model found on Hugging Face!")
        print("   Next steps:")
        print("   1. Run download function to get the model")
        print("   2. Convert URDF to MuJoCo MJCF format")
        print("   3. Add to unitree_mujoco/unitree_robots/")
    elif has_sdk2:
        print("\n[!] A2 URDF not publicly available yet, but SDK2 has A2 support")
        print("   Recommended actions:")
        print("   1. Contact Unitree support: support@unitree.com")
        print("      Request: A2 URDF/MJCF model for MuJoCo simulation")
        print("   2. Monitor Hugging Face for updates:")
        print("      https://huggingface.co/datasets/unitreerobotics/unitree_model")
        print("   3. Alternative: Use B2 model as proxy for testing")
        print("      - B2 is available in unitree_mujoco/unitree_robots/b2/")
        print("      - Similar quadruped platform")
        print("      - Can test flip simulation concepts")
    else:
        print("\n[-] A2 not found anywhere")
        print("   The A2 is a very new robot (2025/2026)")
        print("   Recommended actions:")
        print("   1. Ensure you have latest unitree_sdk2")
        print("   2. Contact Unitree for A2 development resources")
        print("   3. Use existing quadruped (B2/Go2) for prototyping")

def main():
    print("="*60)
    print("Unitree A2 URDF Availability Checker")
    print("="*60)
    
    # Check all sources
    has_hf = check_huggingface_a2()
    has_gh = check_github_unitree_ros()
    has_sdk2 = check_local_sdk2_a2()
    
    # Provide recommendations
    suggest_next_steps(has_hf, has_gh, has_sdk2)
    
    # Offer to download if found
    if has_hf:
        response = input("\n[?] Would you like to download the A2 model now? (y/n): ")
        if response.lower() == 'y':
            download_a2_from_huggingface()
    
    print("\n[+] Check complete!")

if __name__ == "__main__":
    main()
