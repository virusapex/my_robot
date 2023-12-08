import rclpy
from rclpy.node import Node
import subprocess, random
import time, os
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.parameter import Parameter


class ControlMission(Node):
    def __init__(self):
        super().__init__('mission_control')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.getOdom, 1)
        self.sub_robot_finish = self.create_subscription(String, '/robot_finish', self.cbRobotFinish, 1)
        self.traffic_state = 1
        self.loadMissionModel()
        self.setTraffic()
        self.setObstacle()
        self.controlMission()

    def getOdom(self, msg):
        pose_x = msg.pose.pose.position.x
        pose_y = msg.pose.pose.position.y
        # self.get_logger().info(f'Current pose: {pose_x, pose_y}\n {self.traffic_state}')

        # down_bar
        if -2.1 < pose_x < -1.85 and 2.85 < pose_y < 3.05 and self.traffic_state == 5:
            self.traffic_state = 6

        # up_bar
        elif abs(pose_x + 1.3) < 0.15 and (pose_y - 1.25) < 0.05 and self.traffic_state == 7:
            self.traffic_state = 8
            self.current_time = time.time()

    def cbRobotFinish(self, msg):
        self.time_robot_finish = self.get_clock().now().seconds_nanoseconds()
        self.get_logger().info(f"Time for the robot to complete the competition Name_team={msg.data} Ros_now={self.time_robot_finish} Duration={self.time_robot_finish - self.time_robot_start}")
        self.destroy_node()
        rclpy.shutdown()

    def loadMissionModel(self):
        model_dir_path = os.environ.get("GZ_SIM_RESOURCE_PATH").split(":")[-1]

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

        suv_path = model_dir_path + '/suv/suv.sdf'
        with open(suv_path, 'r') as suv:
            self.suv_model = suv.read().replace("\n", "")

        obstacle_path = model_dir_path + '/obstacle.sdf'
        with open(obstacle_path, 'r') as obs:
            self.obstacle_model = obs.read().replace("\n", "")

    def setTraffic(self):
        
        # Find the specific element and change its values
        parking_stop = random.random()
        x = 0.8 if parking_stop < 0.5 else 0.23
        y = 0.8
        z = 0.05
        modified_suv_model = self.suv_model.replace('0 0 0 0 0 -1.57079632679', f'{x} {y} {z} 0 0 0')

        command = ["gz", "service", "-s", "/world/course/create",
                    "--reqtype", "gz.msgs.EntityFactory",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "300",
                    "--req", f'sdf: "{modified_suv_model}"']

        p = subprocess.run(command)

    def setObstacle(self):
        
        # Find the specific element and change its values
        x = random.uniform(-1.39, -0.62)
        y = random.uniform(-1.3, -0.76)
        modified_obstacle_model = self.obstacle_model.replace('-2 -2 0 0 0 0', f'{x} {y} 0 0 0 0')

        command = ["gz", "service", "-s", "/world/course/create",
                    "--reqtype", "gz.msgs.EntityFactory",
                    "--reptype", "gz.msgs.Boolean",
                    "--timeout", "300",
                    "--req", f'sdf: "{modified_obstacle_model}"']

        p = subprocess.run(command)

    def controlMission(self):
        
        if self.traffic_state == 1:  # turn on red light
            
            command = ["gz", "service", "-s", "/world/course/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "300",
                "--req", f'sdf: "{self.red_light_model}"']

            p = subprocess.run(command)
            self.traffic_state = 2
            self.current_time = time.time()

        elif self.traffic_state == 2:
            if abs(self.current_time - time.time()) > random.uniform(1, 3):  # turn on yellow light after 1-3s.
                command = ["gz", "service", "-s", "/world/course/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "300",
                "--req", f'sdf: "{self.yellow_light_model}"']

                p = subprocess.run(command)

                arg = ["gz", "service", "-s", "/world/course/remove",
                "--reqtype", "gz.msgs.Entity",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "300",
                "--req", 'name: "traffic_light_red" type: MODEL']

                p = subprocess.run(arg)
                self.traffic_state = 3
                self.current_time = time.time()

        elif self.traffic_state == 3:
            if abs(self.current_time - time.time()) > random.uniform(4, 7):  # turn on green light after 4-7s.
                command = ["gz", "service", "-s", "/world/course/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "300",
                "--req", f'sdf: "{self.green_light_model}"']

                p = subprocess.run(command)

                arg = ["gz", "service", "-s", "/world/course/remove",
                "--reqtype", "gz.msgs.Entity",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "300",
                "--req", 'name: "traffic_light_yellow" type: MODEL']

                p = subprocess.run(arg)
                self.traffic_state = 4
                self.time_robot_start = self.get_clock().now().seconds_nanoseconds()
                self.get_logger().info(f"Robot start time {self.time_robot_start}")

        elif self.traffic_state == 4: # intersections
            intersection_direction = random.random()

            if intersection_direction < 0.5:
                command = ["gz", "service", "-s", "/world/course/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "300",
                "--req", f'sdf: "{self.traffic_left_model}"']

                p = subprocess.run(command)

            else:
                command = ["gz", "service", "-s", "/world/course/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "300",
                "--req", f'sdf: "{self.traffic_right_model}"']

                p = subprocess.run(command)

            self.traffic_state = 5


def main(args=None):
    rclpy.init(args=args)
    node = ControlMission()
    try:
        while rclpy.ok():
            node.controlMission()
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
