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

## Node launch order control

The basic default offered to control node launch order only guarantee 
you that delayed node is started after target node is launched. 
You have NO GUARANTEE ths target node actually completed 
its initialization, whether it is delayed or may have crashed.
See [documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Event-Handlers.html)
to see if some event handlers can match your need


Here is an example
```commandline
delay_node_after_target = RegisterEventHandler(
    event_handler=OnProcessExit(
        target_action=target_node,
        on_exit=[
            delayed_node
            ],
    ),
)
```

If you need some fine grain control over node launch order, you can rely on state transition
 of Lifecycle nodes.
```commandline
delay_node_after_target = RegisterEventHandler(
    event_handler=OnStateTransition(
        target_lifecycle_node=target_node, goal_state=SOME_TRANSITION,
        entities=[
            delayed_node
            ],
    )
)
```

See [About Lifecycle Node](#about-lifecycle-node) for more information on available transitions.

## Opaque vs standard

You can turn a rosluanch substitution variable into a python varaible through the execution of context 
inside an opaque function
```commandline
var_py = var_rl.perform(context)
```
This can be useful to use more advanced tests on variables.

### Some example
This string format will work as intended in an opaque function, with a 
pythonized, context-executed variable
```commandline
PythonExpression(f"'{var_py}' == 'True'")
```

But this is the way to go in a standard launch context, as the call to 
`PythonExpression` will evaluate `gazebo` later on, with context. 
The former expression will always return False as the string will be 
evaluated before context replacement, on a
`launch.substitutions.launch_configuration.LaunchConfiguration object`
```commandline
PythonExpression(["\"", var_rl, "\" == \"true\""])
```

