# import the necessary packages
import rclpy
from rclpy.node import Node

# import DynamicJointState from the package 'control_msgs'
from control_msgs.msg import DynamicJointState
# import InterfaceValues from the package 'control_msgs'
from control_msgs.msg import InterfaceValue

# import get_fan_trigger_temperature from the package 'reachy_utils'
from reachy_utils.config import get_fan_trigger_temperature, get_reachy_model
from reachy_utils.discovery import robot_config_to_parts


fans_per_part = {
    'head': ['l_antenna_fan', 'r_antenna_fan'],
    'left_arm': ['l_shoulder_fan', 'l_elbow_fan', 'l_wrist_fan'],
    'right_arm': ['r_shoulder_fan', 'r_elbow_fan', 'r_wrist_fan'],
}

motors_to_monitor_per_part = {
    'head': ['l_antenna', 'r_antenna'],
    'left_arm': ['l_shoulder_pitch', 'l_elbow_pitch', 'l_wrist_pitch'],
    'right_arm': ['r_shoulder_pitch', 'r_elbow_pitch', 'r_wrist_pitch'],
}


# Create a ROS2 class to control the fans of the Reachy robot.
class FansController(Node):
# Initialize the node and create the publishers and subscribers.
# In the constructor, add a 'node_name' argument and pass it to the Node constructor.
    def __init__(self, node_name):
        # Call the constructor of the parent class (Node).
        super().__init__(node_name)
        robot_parts = robot_config_to_parts[get_reachy_model()]
        
        # Define the fan trigger temperature from get_fan_trigger_temperature.
        self._fan_trigger_temperature = get_fan_trigger_temperature()

        # Define a dictionnary to store the fan state.
        self._fan_states = {}
        self._motor_temperatures = {}

        for part in robot_parts:
            for motor in motors_to_monitor_per_part[part]:
                self._motor_temperatures[motor] = 0.0

            for fan in fans_per_part[part]:
                self._fan_states[fan] = False

        # Subscribe to the topic 'dynamic_joint_states' the msg type is 'DynamicJointState' from the package 'control_msgs'
        # with queue size 5 and the callback function 'self._on_joint_state'.
        self._joint_state_sub = self.create_subscription(
           DynamicJointState,
          'dynamic_joint_states',
          self._on_joint_state,
         5)

        # Create a publisher with the same topic name 'dynamic_joint_states' and msg type 'DynamicJointState'
        # with queue size 5.
        self._joint_state_pub = self.create_publisher(
            DynamicJointState,
            'dynamic_joint_states',
            5)
        
        # Create a timer with a period of 1 second and the callback function 'self._on_timer'.
        self._timer = self.create_timer(1, self._on_timer)

        # Define a logger attribute to print the fan states.
        self._logger = self.get_logger()


    # Define the callback function '_on_joint_state'.
    def _on_joint_state(self, msg):
        for (joint, interface) in zip(msg.joint_names, msg.interface_values):
            if joint in self._motor_temperatures:
                for (interface_name, value) in zip(interface.interface_names, interface.values):
                    if interface_name == 'temperature':
                        self._motor_temperatures[joint] = value


    # Define the callback function '_on_timer'.
    def _on_timer(self):
        # Iterate over each motor and check if the temperature is above the fan trigger temperature.
        for motor, temperature in self._motor_temperatures.items():
            # split the motor name with 'pitch' as separator and get the first element.
            # Example: 'l_shoulder_pitch' -> 'l_shoulder'
            motor = motor.split('_pitch')[0]

            if temperature >= self._fan_trigger_temperature:
                # Set the fan state to True.
                self._fan_states[f'{motor}_fan'] = True
            
            # else if temperature is below the fan trigger temperature -2°C, set the fan state to False.
            elif temperature < self._fan_trigger_temperature - 2:
                # Get the fan name from the motor name by replacing 'pitch' with 'fan'.
                # Example: 'l_shoulder_pitch' -> 'l_shoulder_fan'
                self._fan_states[f'{motor}_fan'] = False

        # Print the fan states and the motor temperatures.
        self._logger.info(f'Fans state: {self._fan_states}')
        self._logger.info(f'Motors temperature: {self._motor_temperatures}')

        # Create a DynamicJointState msg.
        msg = DynamicJointState()
        msg.joint_names = self._fan_states.keys()
        # Iterate over each fan state and set the interface values.
        # Example: {'l_shoulder_fan': True, 'r_shoulder_fan': False} -> [InterfaceValue(interface_names=['state'], values=[1.0]), InterfaceValue(interface_names=['state'], values=[0.0])]
        msg.interface_values = [
            InterfaceValue(
                interface_names=['state'],
                values=[float(state)],
            )
            for state in self._fan_states.values()
        ]

        # Publish the msg.
        self._joint_state_pub.publish(msg)


# Create the main function to initialize the node and call the class.
def main(args=None):
    # Initialize the rclpy library.
    rclpy.init(args=args)
    # Create the node.
    node = FansController('fans_controller')
    # Spin the node.
    rclpy.spin(node)
    # Destroy the node.
    node.destroy_node()
    # Shutdown the ROS2 communication.
    rclpy.shutdown()
