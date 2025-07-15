import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer
import yaml
import os
import time

class ViconStringRecorder(Node):
    def __init__(self):
        super().__init__('vicon_string_recorder')

        # tf2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Declare parameters
        self.declare_parameter('string_ends', ['string1_start', 'string1_end', 
                                               'string2_start', 'string2_end'])
        self.declare_parameter('output_file', 'string_obstacles.yaml')
        self.declare_parameter('duration', 5.0)

        self.string_ends = self.get_parameter('string_ends').get_parameter_value().string_array_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.duration = self.get_parameter('duration').get_parameter_value().double_value

        self.positions = {}
        self.start_time = self.get_clock().now()

        # Timer to check if time elapsed
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        now = self.get_clock().now()
        for name in self.string_ends:
            try:
                trans = self.tf_buffer.lookup_transform('world', name, rclpy.time.Time())
                pos = trans.transform.translation
                self.positions[name] = [pos.x, pos.y, pos.z]
            except Exception as e:
                self.get_logger().warn(f'Could not get transform for {name}: {str(e)}')

        if (now - self.start_time).nanoseconds / 1e9 > self.duration:
            self.write_output()
            rclpy.shutdown()

    def write_output(self):
        strings = []
        # Pair each start/end
        for i in range(0, len(self.string_ends), 2):
            start_name = self.string_ends[i]
            end_name = self.string_ends[i+1]
            if start_name in self.positions and end_name in self.positions:
                strings.append({
                    'start': self.positions[start_name],
                    'end': self.positions[end_name]
                })
            else:
                self.get_logger().warn(f'Skipping incomplete pair: {start_name}, {end_name}')

        data = {'strings': strings}
        with open(self.output_file, 'w') as f:
            yaml.dump(data, f)
        self.get_logger().info(f"Written string obstacle data to {self.output_file}")

def main():
    rclpy.init()
    node = ViconStringRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()