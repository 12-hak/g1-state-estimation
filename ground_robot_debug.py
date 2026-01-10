    def _ground_robot(self):
        """Adjust base height so feet stay on the ground."""
        # First, set a high initial height so feet are above ground
        self.data.qpos[2] = 2.0

        # Run forward kinematics to get body positions
        mujoco.mj_forward(self.model, self.data)

        # Try multiple methods to find foot positions
        lowest_z = None
        
        # Method 1: Try foot sites
        try:
            left_foot_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "left_foot")
            right_foot_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "right_foot")
            
            if left_foot_id >= 0 and right_foot_id >= 0:
                left_z = self.data.site_xpos[left_foot_id, 2]
                right_z = self.data.site_xpos[right_foot_id, 2]
                lowest_z = min(left_z, right_z) - 0.025
                print(f"[Ground] Using foot sites: L={left_z:.3f}, R={right_z:.3f}")
        except:
            pass
        
        # Method 2: Try ankle bodies
        if lowest_z is None:
            try:
                left_ankle_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "left_ankle_link")
                right_ankle_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "right_ankle_link")
                
                if left_ankle_id >= 0 and right_ankle_id >= 0:
                    left_z = self.data.xpos[left_ankle_id, 2]
                    right_z = self.data.xpos[right_ankle_id, 2]
                    lowest_z = min(left_z, right_z) - 0.05
                    print(f"[Ground] Using ankle bodies: L={left_z:.3f}, R={right_z:.3f}")
            except:
                pass
        
        # Method 3: Scan all bodies for lowest point
        if lowest_z is None:
            print("[Ground] Scanning all bodies for lowest point...")
            for i in range(self.model.nbody):
                body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, i)
                if body_name and ('ankle' in body_name.lower() or 'foot' in body_name.lower()):
                    z = self.data.xpos[i, 2]
                    if lowest_z is None or z < lowest_z:
                        lowest_z = z - 0.05
                    print(f"  Found: {body_name} at z={z:.3f}")
        
        # Apply grounding
        if lowest_z is not None:
            self.data.qpos[2] -= lowest_z
            print(f"[Ground] Final base height: {self.data.qpos[2]:.3f}")
        else:
            self.data.qpos[2] = 0.793
            print("[Ground] WARNING: Using default height 0.793")
