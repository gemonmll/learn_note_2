---

c_cpp_ propeties.json 

```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/humble/include/**"  // <--- 添加这一行
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c17",
            "cppStandard": "c++17",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
```

---

launch.json

```json
{
    "version": "0.2.0",
    "configurations": [
      {
        "name": "ROS2 Debug - minimal_client",
        "type": "cppdbg",
        "request": "launch",
        "program": "/workspace/my_learning/install/cpp_srvcli/lib/cpp_srvcli/server",
        "args": [],
        "stopAtEntry": false,
        "cwd": "${workspaceFolder}",
        "environment": [
          {
            "name": "AMENT_PREFIX_PATH",
            "value": "/opt/ros/humble:/workspace/my_learning/install"
          },
          {
            "name": "COLCON_PREFIX_PATH",
            "value": "/opt/ros/humble:/workspace/my_learning/install"
          },
          {
            "name": "LD_LIBRARY_PATH",
            "value": "/opt/ros/humble/lib:/workspace/my_learning/install/cpp_srvcli/lib:/workspace/my_learning/install/lib"
          },
          {
            "name": "RMW_IMPLEMENTATION",
            "value": "rmw_fastrtps_cpp"
          }
        ],
        "externalConsole": true,
        "MIMode": "gdb",
        "setupCommands": [
          {
            "description": "Enable pretty-printing for gdb",
            "text": "-enable-pretty-printing",
            "ignoreFailures": true
          }

        ],
        "preLaunchTask": "source_ros2_env"
      }
    ]
  }
  

```

---
task.json

```json

{
"version": "2.0.0",
"tasks": [
    {
        "label": "source_ros2_env",
        "type": "shell",
        "command": "bash -c 'source /opt/ros/humble/setup.bash && source /workspace/my_learning/install/setup.bash'"
        }
]
}
  

```