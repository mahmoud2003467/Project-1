#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn
import math
import random
from functools import partial

class TurtleHunter(Node):
    def __init__(self):
        super().__init__('turtle_hunter')

       
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        
        self.turtle1_pose = None
        self.other_turtles = {}  # {"turtle2": pose, ...}
        self.spawned_count = 5   # total turtles including turtle1
        self.max_turtles = 5

       
        self.kill_client = self.create_client(Kill, '/kill')
        self.spawn_client = self.create_client(Spawn, '/spawn')

        self.kill_client.wait_for_service()
        self.spawn_client.wait_for_service()

        
        self.create_subscription(Pose, '/turtle1/pose', self.turtle1_callback, 10)

        
        initial_turtle_names = ["turtle2", "turtle3", "turtle4", "turtle5"]
        for name in initial_turtle_names:
            self.subscribe_to_turtle(name)

        self.get_logger().info("Turtle1 will hunt and replace turtles!")

        
        self.create_timer(0.1, self.control_loop)

    def subscribe_to_turtle(self, name):
        """Subscribe to a turtle pose and track it."""
        self.create_subscription(
            Pose,
            f'/{name}/pose',
            partial(self.other_turtle_callback, name=name),
            10
        )

    def turtle1_callback(self, msg: Pose):
        self.turtle1_pose = msg

    def other_turtle_callback(self, msg: Pose, name: str):
        self.other_turtles[name] = msg

    def control_loop(self):
        if self.turtle1_pose is None or not self.other_turtles:
            return

        
        closest_name, closest_pose, min_distance = self.get_closest_turtle()
        if closest_pose is None:
            return

        
        dx = closest_pose.x - self.turtle1_pose.x
        dy = closest_pose.y - self.turtle1_pose.y
        angle_to_target = math.atan2(dy, dx)
        angle_diff = angle_to_target - self.turtle1_pose.theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        cmd = Twist()

        if min_distance > 0.5:
            # Chase target
            cmd.linear.x = 2.0
            cmd.angular.z = 4.0 * angle_diff
            self.cmd_vel_pub.publish(cmd)
        else:
            
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info(f"Target {closest_name} caught! Killing...")
            self.kill_turtle(closest_name)

    def get_closest_turtle(self):
        closest_name = None
        closest_pose = None
        min_distance = float('inf')

        for name, pose in self.other_turtles.items():
            dx = pose.x - self.turtle1_pose.x
            dy = pose.y - self.turtle1_pose.y
            distance = math.sqrt(dx**2 + dy**2)
            if distance < min_distance:
                min_distance = distance
                closest_name = name
                closest_pose = pose

        return closest_name, closest_pose, min_distance

    def kill_turtle(self, name):
        """Kill a turtle and spawn another one."""
        request = Kill.Request()
        request.name = name
        future = self.kill_client.call_async(request)
        future.add_done_callback(partial(self.callback_kill, name=name))

    def callback_kill(self, future, name):
        try:
            future.result()
            self.get_logger().info(f"Turtle {name} killed!")
            if name in self.other_turtles:
                del self.other_turtles[name]

            # Spawn another turtle if we have less than max
            if len(self.other_turtles) < self.max_turtles - 1:
                self.spawn_new_turtle()
        except Exception as e:
            self.get_logger().error(f"Failed to kill {name}: {e}")

    def spawn_new_turtle(self):
        """Spawn a new turtle at a random location."""
        self.spawned_count += 1
        new_name = f"turtle{self.spawned_count}"

        x = random.uniform(1.0, 10.0)
        y = random.uniform(1.0, 10.0)
        theta = random.uniform(0.0, 6.28)

        req = Spawn.Request()
        req.x = x
        req.y = y
        req.theta = theta
        req.name = new_name

        future = self.spawn_client.call_async(req)
        future.add_done_callback(partial(self.callback_spawn, name=new_name))

    def callback_spawn(self, future, name):
        try:
            response = future.result()
            self.get_logger().info(f"Spawned new turtle: {response.name}")
            self.subscribe_to_turtle(name)
        except Exception as e:
            self.get_logger().error(f"Failed to spawn turtle: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleHunter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
