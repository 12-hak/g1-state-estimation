import os
import requests

base_url = "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/master/robots/a2_description/meshes/"
target_dir = r"d:\python\state_e\unitree_mujoco\unitree_robots\a2_official\meshes"

meshes = [
    "base_link.STL",
    "left_front_Link1.STL", "left_front_Link2.STL", "left_front_Link3.STL", "left_front_Link4.STL",
    "left_hind_Link1.STL", "left_hind_Link2.STL", "left_hind_Link3.STL", "left_hind_Link4.STL",
    "right_front_Link1.STL", "right_front_Link2.STL", "right_front_Link3.STL", "right_front_Link4.STL",
    "right_hind_Link1.STL", "right_hind_Link2.STL", "right_hind_Link3.STL", "right_hind_Link4.STL"
]

if not os.path.exists(target_dir):
    os.makedirs(target_dir)

for mesh in meshes:
    url = base_url + mesh
    path = os.path.join(target_dir, mesh)
    print(f"Downloading {mesh}...")
    try:
        r = requests.get(url)
        with open(path, 'wb') as f:
            f.write(r.content)
    except Exception as e:
        print(f"Failed to download {mesh}: {e}")

print("Done.")
