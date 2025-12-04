import rclpy
from rclpy.node import Node
import subprocess, random
import time, os
import cv2
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32
from rclpy.parameter import Parameter


class ControlMission(Node):
    '''
    Referee node that controls mission spawn and calculates time when
    robot finishes the race.

    Subscriptions:
        /odom           Odometry of the robot
        /robot_finish   Should receive the name of the team and print final time

    Parameters:
        use_sim_time    Forces the node to use Gazebo internal clock
    '''
    def __init__(self):
        super().__init__('mission_control')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.sub_odom = self.create_subscription(Odometry,
                                                 '/odom',
                                                 self.getOdom,
                                                 1)
        self.sub_robot_finish = self.create_subscription(String,
                                                         '/robot_finish',
                                                         self.cbRobotFinish,
                                                         1)
        self.sub_aruco_mission = self.create_subscription(Float32,
                                                          '/mission_aruco',
                                                          self.cbArucoMission,
                                                          1)
        self.aruco_marker_id = -1
        self.traffic_state = 1  # initial state
        self.loadMissionModel() # pre-loads all necessary models
        self.controlMission()   # spawns the traffic light and starts the timer

    def getOdom(self, msg):
        '''
        Listens for odometry messages, might be useful to create
        dynamic missions. Currently, auto detects if robot crosses
        the finish line and stops the timer.
        '''
        pose_x = msg.pose.pose.position.x
        pose_y = msg.pose.pose.position.y
        # Un-comment to print current odometry pose
        # self.get_logger().info(f'Current pose: {pose_x, pose_y}\n {self.traffic_state}')

        if -0.12 < pose_x < 0.12 and -0.12 < pose_y < 0.12 and self.traffic_state == 6:
            self.autoRobotFinish()  # auto detect finish

    def convert_to_float(self, time_tuple):
        '''
        Helper method to convert ROS2 time to float
        '''
        seconds, nanoseconds = time_tuple
        total_seconds = seconds + nanoseconds / 1e9
        return total_seconds

    def autoRobotFinish(self):
        '''
        Automatically detects if robot has crossed the finish line
        '''
        self.time_robot_finish = self.convert_to_float(self.get_clock().now()
                                                       .seconds_nanoseconds())
        self.get_logger().info(f"Sim_time={self.time_robot_finish:.3f} \n \
                               Final_time={(self.time_robot_finish - self.time_robot_start):.3f}")
        self.destroy_node()

    def cbRobotFinish(self, msg):
        '''
        In case the team decides to end the race
        (couldn't complete some of the missions)
        they can stop the timer by sending a string to /robot_finish topic
        '''
        self.time_robot_finish = self.convert_to_float(self.get_clock().now()
                                                       .seconds_nanoseconds())
        self.get_logger().info(f"Team_name={msg.data} \n \
                               Sim_time={self.time_robot_finish:.3f} \n \
                               Final_time={(self.time_robot_finish - self.time_robot_start):.3f}")
        self.destroy_node()

    def cbArucoMission(self, msg):
        '''
        Callback for AruCo mission verification
        '''
        if self.aruco_marker_id == -1:
            self.get_logger().warn("AruCo marker not yet generated!")
            return

        expected_value = np.sqrt(self.aruco_marker_id)
        if abs(msg.data - expected_value) < 1e-3:
            self.get_logger().info("Mission AruCo Success")
        else:
            self.get_logger().info(f"Mission AruCo Fail: Expected {expected_value:.3f}, got {msg.data:.3f}")

    def loadMissionModel(self):
        '''
        Pre-loads assets for the missions
        '''
        model_dir_path = os.environ.get("GZ_SIM_RESOURCE_PATH").split(":")[0]

        red_light_path = model_dir_path + '/traffic_light/red.sdf'
        with open(red_light_path, 'r') as rlm:
            self.red_light_model = rlm.read().replace("\n", "")

        yellow_light_path = model_dir_path + '/traffic_light/yellow.sdf'
        with open(yellow_light_path, 'r') as ylm:
            self.yellow_light_model = ylm.read().replace("\n", "")

        green_light_path = model_dir_path + '/traffic_light/green.sdf'
        with open(green_light_path, 'r') as glm:
            self.green_light_model = glm.read().replace("\n", "")

        traffic_left_path = model_dir_path + '/intersection/left.sdf'
        with open(traffic_left_path, 'r') as tlm:
            self.traffic_left_model = tlm.read().replace("\n", "")

        traffic_right_path = model_dir_path + '/intersection/right.sdf'
        with open(traffic_right_path, 'r') as trm:
            self.traffic_right_model = trm.read().replace("\n", "")

        self.aruco_sign_model = """
<sdf version='1.10'>
    <model name='aruco_sign'>
      <static>true</static>
      <link name='box'>
        <pose>-0.88 1.05 0.13 0 0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>0.05 0.15 0.15</size>
            </box>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>0.05 0.15 0.15</size>
            </box>
          </geometry>
          <material>
            <diffuse>1 1 1 1</diffuse>
            <specular>1 1 1 1</specular>
            <pbr>
              <metal>
                <albedo_map>/tmp/aruco_marker.png</albedo_map>
              </metal>
            </pbr>
          </material>
        </visual>
      </link>
    </model>
</sdf>
""".replace('\n', '')

    def generateArucoMarker(self):
        '''
        Generates a random AruCo marker from DICT_6x6_250
        '''
        marker_size = 200
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_marker_id = random.randint(0, 249)
        marker_img = cv2.aruco.drawMarker(aruco_dict, self.aruco_marker_id, marker_size)
        border_size = marker_size // 6
        marker_with_border = cv2.copyMakeBorder(
            marker_img, 
            top=border_size, 
            bottom=border_size, 
            left=border_size, 
            right=border_size, 
            borderType=cv2.BORDER_CONSTANT, 
            value=255
        )
        cv2.imwrite('/tmp/aruco_marker.png', marker_with_border)
        self.get_logger().info(f"Generated AruCo marker with ID: {self.aruco_marker_id}")

    def spawnArucoSign(self):
        '''
        Spawns the sign with the generated AruCo marker
        '''
        self.generateArucoMarker()
        
        command = ["gz", "service", "-s", "/world/course/create",
                    "--reqtype", "gz.msgs.EntityFactory",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "300",
                    "--req", f'sdf: "{self.aruco_sign_model}"']
        
        subprocess.run(command)

    def controlMission(self):
        '''
        Controls current states and changes the traffic light
        '''
        if self.traffic_state == 1:  # turn on red light
            # Calls a service to spawn the model
            command = ["gz", "service", "-s", "/world/course/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "300",
                "--req", f'sdf: "{self.red_light_model}"']

            p = subprocess.run(command)
            self.traffic_state = 2
            self.current_time = time.time()

        elif self.traffic_state == 2:  # turn on yellow light after 1-3s.
            if abs(self.current_time - time.time()) > random.uniform(1, 3):
                # Calls a service to spawn the model
                command = ["gz", "service", "-s", "/world/course/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "300",
                "--req", f'sdf: "{self.yellow_light_model}"']

                p = subprocess.run(command)
                # Removes previous model
                arg = ["gz", "service", "-s", "/world/course/remove",
                "--reqtype", "gz.msgs.Entity",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "300",
                "--req", 'name: "traffic_light_red" type: MODEL']

                p = subprocess.run(arg)
                self.traffic_state = 3
                self.current_time = time.time()

        elif self.traffic_state == 3:   # turn on green light after 4-7s.
            if abs(self.current_time - time.time()) > random.uniform(4, 7):
                # Calls a service to spawn the model
                command = ["gz", "service", "-s", "/world/course/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "300",
                "--req", f'sdf: "{self.green_light_model}"']

                p = subprocess.run(command)
                # Removes previous model
                arg = ["gz", "service", "-s", "/world/course/remove",
                "--reqtype", "gz.msgs.Entity",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "300",
                "--req", 'name: "traffic_light_yellow" type: MODEL']

                p = subprocess.run(arg)
                self.traffic_state = 4
                # Green light indicates the beginning of the race!
                self.time_robot_start = self.convert_to_float(self.get_clock().now()
                                                              .seconds_nanoseconds())
                self.get_logger().info(f"Robot start time {self.time_robot_start:.3f}")
                
                self.spawnArucoSign()

        elif self.traffic_state == 4: # intersections
            intersection_direction = random.random()

            if intersection_direction < 0.5:
                # Calls a service to spawn the model
                command = ["gz", "service", "-s", "/world/course/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "300",
                "--req", f'sdf: "{self.traffic_left_model}"']

                p = subprocess.run(command)

            else:
                # Calls a service to spawn the model
                command = ["gz", "service", "-s", "/world/course/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "300",
                "--req", f'sdf: "{self.traffic_right_model}"']

                p = subprocess.run(command)

            self.traffic_state = 5


def main(args=None):
    '''
    Creates the class object and spins subscriptions
    '''
    rclpy.init(args=args)
    node = ControlMission()
    try:
        while rclpy.ok():
            node.controlMission()
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

if __name__ == '__main__':
    main()
