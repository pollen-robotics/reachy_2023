Gripper Glue

A gazebo plugin that force link creation on certain conditions, to ease grasp simulation

# Installation
Add this to your bashrc, with the right PATH

```
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:"YOURPATHTOYOURREACHYWS"/install/reachy_gazebo_gripper_glue/lib


```


everything else shall be handled by colcon

# Usage

add this to your world to load the plugin into it
```commandline
<plugin name="reachy_gazebo_gripper_glue" filename="libreachy_gazebo_gripper_glue.so"/>

```


il y a un petit comment `// LOOK AT ME, I'M THE GLUE NOW` pour indiquer le principal levier de mofi à savoir la condition de trigger du glue
