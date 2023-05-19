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
        self._logger = self.get_logger()

        robot_parts = robot_config_to_parts[get_reachy_model()]
        
        # Define the fan trigger temperature from get_fan_trigger_temperature. 
        self._fan_trigger_temperature = get_fan_trigger_temperature()
        if self._fan_trigger_temperature == -1:
            # print an error message if the fan trigger temperature is not defined.
            # and exit the program.
            self.get_logger().error('Fan trigger temperature is not defined.')
            exit()

        # print self._fan_trigger_temperature
        self._logger.info(f'Fan trigger temperature is {self._fan_trigger_temperature}°C.')

        # Define a dictionnary to store the fan state.
        self._fan_states = {}
        self._fan_states_to_publish = []
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
            'dynamic_joint_commands',
            5)

        self._logger.info('Fans controller node started.')

        self._handle_first_message()

        # Create a timer with a period of 1 second and the callback function 'self._on_timer'.
        self._timer = self.create_timer(1, self._on_timer)

        # Define a logger attribute to print the fan states.

        # Log a message to indicate that the node is started.

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

            if temperature >= self._fan_trigger_temperature and not self._fan_states[f'{motor}_fan']:
                # Set the fan state to True.
                # Log the fan name and the motor temperature.
                # self._logger.info(f'{motor}_fan: {temperature}')

                self._fan_states[f'{motor}_fan'] = True
                self._fan_states_to_publish.append(f'{motor}_fan')
            
            # else if temperature is below the fan trigger temperature -2°C, set the fan state to False.
            elif temperature < self._fan_trigger_temperature - 2 and self._fan_states[f'{motor}_fan']:
                # Get the fan name from the motor name by replacing 'pitch' with 'fan'.
                # Example: 'l_shoulder_pitch' -> 'l_shoulder_fan'
                self._fan_states[f'{motor}_fan'] = False
                self._fan_states_to_publish.append(f'{motor}_fan')

        # Create a DynamicJointState msg.
        if self._fan_states_to_publish:
            msg = DynamicJointState()
            msg.joint_names = self._fan_states_to_publish
            # Iterate over each fan state and set the interface values.
            # Example: {'l_shoulder_fan': True, 'r_shoulder_fan': False} -> [InterfaceValue(interface_names=['state'], values=[1.0]), InterfaceValue(interface_names=['state'], values=[0.0])]
            msg.interface_values = [
                InterfaceValue(
                    interface_names=['state'],
                    values=[float(self._fan_states[fan])],
                )
                for fan in self._fan_states_to_publish
            ]

            # Publish the msg.
            self._joint_state_pub.publish(msg)


            for fan in self._fan_states_to_publish:
                state = 'ON' if self._fan_states[fan] else 'OFF'
                self._logger.info(f'Turning {state} {fan}.')

            self._fan_states_to_publish = []
            # Print the msg.
            # self._logger.info(f'Publishing fan states: {msg}')

    def _handle_first_message(self):
        for motor, temperature in self._motor_temperatures.items():
            motor = motor.split('_pitch')[0]

            if temperature >= self._fan_trigger_temperature:
                self._fan_states[f'{motor}_fan'] = True

            elif temperature < self._fan_trigger_temperature - 2 and self._fan_states[f'{motor}_fan']:
                self._fan_states[f'{motor}_fan'] = False

        msg = DynamicJointState()
        msg.joint_names = list(self._fan_states.keys())
        msg.interface_values = [
            InterfaceValue(
                interface_names=['state'],
                values=[float(state)],
            )
            for state in self._fan_states.values()
        ]
        self._joint_state_pub.publish(msg)
        self._logger.info(f'Sent initial fan command:')
        for (fan, state) in self._fan_states.items():
            state = 'ON' if state else 'OFF'
            self._logger.info(f'    {fan} is {state}')


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
