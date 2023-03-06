# About LifeCycle Node

Here is the [state machine](https://design.ros2.org/articles/node_lifecycle.html
)

ROS launch file [example](https://github.com/ros2/launch_ros/blob/6370c127868a5056a8a02c9412c59bebdaefcf81/launch_ros/examples/lifecycle_pub_sub_launch.py#L59)

Python node file [example](https://github.com/ros2/demos/blob/rolling/lifecycle_py/lifecycle_py/talker.py)



# Some notes on .launch.py

## Log

```commandline
from launch.actions import LogInfo
LogInfo(msg="Some nice mesage {}".format(arg_py)).execute(context=context)
```

## Opaque vs standard
This string format will work as intended in an opaque function, with a 
pythonized, context-exececuted variable
```commandline
PythonExpression(f"'{gazebo_py}' == 'true'")
```

But this is the way to go in a standard launch context, as the call to 
`PythonExpression` will evaluate `gazebo` later on, with context. 
The former expression will always return False as the string will be 
evaluated before context replacement, on a
`launch.substitutions.launch_configuration.LaunchConfiguration object`
```commandline
PythonExpression(["\"", gazebo, "\" == \"true\""])
```
