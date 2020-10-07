Notice: After the recent Carla update 0.9.10, there are errors in the build which is being resolved. The code will work for 0.9.9

# CARLA-Racing

A program for a self-driving car to learn to drive faster on a racetrack. The 3D environment is CARLA, an open source autonomous driving simulator. We use vehicle sensors like, for example, RGB camera, RADAR, LIDAR, IMU, etc. for accurate state estimation and, for the time being, Learning Model Predictive Controller (LMPC) that decides the control inputs and possible future states.

See https://github.com/urosolia/RacingLMPC for LMPC implementation.
## Requirements (CARLA Packaged):
See https://github.com/carla-simulator/carla/releases for download.
1. If you are running the packaged version, and not building from source, simply download and extract the repository in the `examples` folder in `PythonAPI`.
2. Run `CarlaUE4.exe`. This will start the server in a game window.
3. Run `commander.py` in Python 3.7 and observe the game window.

## Post-setup Instructions (CARLA Packaged):
Currently the simulation is coded on `Town07` map. By default the Unreal Editor loads `Town03`. `Town07` is an additional map and is not included with the base package. The additional maps can be downloaded from https://github.com/carla-simulator/carla/releases. Extract the `.rar` or `.zip` in [root-CARLA folder] and overwrite on prompt. Before starting `CarlaUE4.exe`, go to [root-Carla folder] -> CarlaUE4 -> Config. Open `DefaultEngine.ini` with Notepad. Change `Town03` to `Town07` for all assignments. The Unreal Editor will now load the project with Town07.

## Requirements (CARLA from source):
1. CARLA built from source (Wondows and/or Linux). See https://carla.readthedocs.io/en/latest/build_windows/ for installation instructions. It is highly advisable to strictly follow the instructions for successful installation. Note that You should have both Windows 10 SDK and Windows 8.1 SDK for VS 2017.
2. Unreal Engine 4.24.3 with debugging symbols add-on. This can be done via Epic Games Launcher (https://www.epicgames.com/site/en-US/home)
3. Python 3.7 with OpenCV, NumPy, SciPy and CvxOpt.
4. Dedicated GPU with at least 4 GB VRAM (especially for Machine Learning).
5. Minimum 40 GB HDD space.
6. Minimum 8 GB DDR3 RAM

## Post-setup Instructions (CARLA from source):
Currently the simulation is coded on Town07 map. By default the Unreal Editor loads Town03. Close the Unreal Editor, and exit the Epic Games Launcher. Go to [Carla-master folder] -> Unreal -> CarlaUE4 -> Config. Open DefaultEngine.ini with Notepad. Change Town03 to Town07 for all assignments. The Unreal Editor will now load the project with Town07.
