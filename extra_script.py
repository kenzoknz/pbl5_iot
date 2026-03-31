import os
Import("env")

# Force micro_ros_platformio to use bash on Windows
if os.name == "nt":
    env["ENV"]["SHELL"] = "C:/Program Files/Git/bin/bash.exe"