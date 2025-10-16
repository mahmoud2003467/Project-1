#!/usr/bin/env python3

import rclpy
import random
import time
from rclpy.node import Node
from turtlesim.srv import Spawn


class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('spawner')
        self.client = self.create_client(Spawn, '/spawn')
        while not self.client.wait_for_service(1.0):
            self.get_logger().info("Waiting for service ...")

        num_turtles = 4
        i = 0

        while i < num_turtles :
            x = random.uniform(1.0,10.0)
            y = random.uniform(1.0,10.0)
            theta = random.uniform(0.0,6.28)
            name = f"turtle{i+2}"
            i = i+1
            self.spawn_turtle(x,y,theta,name)
            time.sleep(3)
    
    
    def spawn_turtle(self,x,y,theta,name):
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = self.client.call_async(request)
        future.add_done_callback(self.callback_spawn)

    def callback_spawn(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Spawned new turtle: {response.name}")
        except Exception as e:
            self.get_logger().error(f"service call failed: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

        