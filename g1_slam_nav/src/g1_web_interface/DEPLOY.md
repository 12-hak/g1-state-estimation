# Deploying the web interface to the robot

## Build and serve (so the robot uses your latest frontend)

1. **Copy** the whole `g1_web_interface` directory to the robot (e.g. into your ROS workspace `src/`).

2. **Build the frontend** (on the robot or on your PC before copying):
   ```bash
   cd g1_web_interface/frontend
   npm install
   npm run build
   ```
   This writes output to `frontend/dist/` (Vite default).

3. **Install the ROS package** so the bridge serves that `dist`:
   ```bash
   cd /path/to/workspace
   colcon build --packages-select g1_web_interface
   source install/setup.bash
   ```
   `setup.py` packages `frontend/dist` into `share/g1_web_interface/frontend/`, which the bridge uses.

4. **Run the bridge** (e.g. via your usual launch). It will serve from `share/.../frontend` (installed dist).

5. **Hard-refresh the browser** (Ctrl+Shift+R or Cmd+Shift+R) so the new JS bundle loads. If you still see old behaviour, clear cache or try an incognito window.

## Check you’re on the new build

- In the UI you should see **“· no decay”** next to the connection status. If you don’t, the browser is likely serving a cached bundle.

## If running from source (no install)

- The node looks for `src/g1_web_interface/frontend/dist` under the **current working directory**. Run the launch from your workspace root so that path exists after `npm run build`.
