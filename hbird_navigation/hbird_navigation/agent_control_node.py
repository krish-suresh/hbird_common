<<<<<<< HEAD
import math
=======
>>>>>>> b58f4bc620eb432bfe0c93ac6faefbc4569bbd66
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

import pathlib
import yaml

from hbird_msgs.msg import Waypoint, State
from time import perf_counter
from enum import Enum, auto

class MovementState(Enum):
    GROUND=auto()
    TAKEOFF=auto()
    SW =auto()
    NE = auto()
    NW = auto()
    SE = auto()


class AgentControlNode(Node):
    
    def __init__(self):
        super().__init__('agent_control_node')

        self.get_logger().info('Starting up the Agent Control Node...')

        # TODO: Add the parameter getter and the parameter declaration
        param_descriptor = ParameterDescriptor(description='Defines agent ID.')
        self.declare_parameter('agent_id', 'HB1', param_descriptor)
        self._agent_id = self.get_parameter('agent_id')._value
        
        # define ROS2 topics
        vehicle_state_topic = '/'+self._agent_id+'/agent_state'
        pos_setpoint_topic = '/'+self._agent_id+'/position_setpoint'

        # initialize subscriber and publisher
        self._state_subscriber = self.create_subscription(State, 
                                                          vehicle_state_topic,
                                                          self.state_update_callback, 10)

        self._pos_setpoint_publisher = self.create_publisher(Waypoint, 
                                                             pos_setpoint_topic, 10)
        
        # initialize timer
        self._publish_rate = 0.2  # sec/cycle
        self._publish_timer = self.create_timer(self._publish_rate, self.control_cycle)

        # define stage
        self.stage : MovementState = MovementState.GROUND
        self._state = State()
        # get planner config file
        config_path = str(pathlib.Path().parent.resolve())+"/src/hbird_common/hbird_navigation/hbird_navigation/config/"
        config_file = "planner_config.yaml"
        try:
            with open(config_path+config_file, "r") as file:
                config = yaml.load(file, Loader=yaml.FullLoader)
        except FileNotFoundError:
            print(f"The file '{config_file}' does not exist.")
        except yaml.YAMLError as e:
            print(f"Error parsing the YAML file: {e}")

        # ---------------------------------------------

        # add functions from planner_main.py here below

        # ---------------------------------------------
        

        # set thresholds
        self.pos_threshold = 0.1
        self.orient_threshold = 0.05
        self.state_time = perf_counter()

    def state_update_callback(self, state_msg):
        self._state = state_msg

    
    def control_cycle(self):

        pos_setpoint = Waypoint()

        self.get_logger().info(f"State: {self.stage.name}, Time {(perf_counter() - self.state_time):.2f}")
        match self.stage:
            case MovementState.GROUND:
                pos_setpoint.position.z = 0.0
                pos_setpoint.heading = math.pi
                if perf_counter() - self.state_time > 5:
                    self.state_time = perf_counter()
                    self.stage = MovementState.TAKEOFF
            case MovementState.TAKEOFF:
                MovementState.GROUND.name
                pos_setpoint.position.z = 1.0
                pos_setpoint.heading = math.pi
                if perf_counter() - self.state_time > 5:
                    self.stage = MovementState.SW
            case MovementState.SW:
                pos_setpoint.position.x = 1.0
                pos_setpoint.position.y = 1.0
                pos_setpoint.position.z = 1.0
                pos_setpoint.heading = math.pi
                if perf_counter() - self.state_time > 5:
                    self.state_time = perf_counter()
                    self.stage = MovementState.NE
            case MovementState.NE:
                pos_setpoint.position.x = -2.0
                pos_setpoint.position.y = -2.0
                pos_setpoint.position.z = 1.0
                pos_setpoint.heading = math.pi
                if perf_counter() - self.state_time > 5:
                    self.state_time = perf_counter()
                    self.stage = MovementState.SE
            case MovementState.SE:
                pos_setpoint.position.x = 2.0
                pos_setpoint.position.y = -2.0
                pos_setpoint.position.z = 1.0
                pos_setpoint.heading = math.pi
                if perf_counter() - self.state_time > 5:
                    self.state_time = perf_counter()
                    self.stage = MovementState.NW
            case MovementState.NW:
                pos_setpoint.position.x = -2.0
                pos_setpoint.position.y = 2.0
                pos_setpoint.position.z = 1.0
                pos_setpoint.heading = math.pi
                if perf_counter() - self.state_time > 5:
                    self.state_time = perf_counter()
                    self.stage = MovementState.TAKEOFF



        # publish the setpoint
        self._pos_setpoint_publisher.publish(pos_setpoint)

    
    def map_to_webots_transform(self, waypoint):
        """Helper function that returns a Waypoint adjusted to the webots reference frame"""
        webots_wp = Waypoint()
        webots_wp.position.x = waypoint.position.x - 4.5
        webots_wp.position.y = waypoint.position.y - 5.0
        return webots_wp



def main(args=None):
    rclpy.init(args=args)

    agent_control = AgentControlNode()

    rclpy.spin(agent_control)

    # Destroy the node explicitly
    agent_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
