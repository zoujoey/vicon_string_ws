#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import yaml
import os
from collections import defaultdict

class StringTransformAverager(Node):
    def __init__(self):
        super().__init__('string_transform_averager')

        # Number of strings (i.e., string11/string12 → stringn1/stringn2)
        self.num_strings = 1  # Adjust this as needed

        # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Storage for readings
        self.readings = defaultdict(list)
        self.max_samples = 10

        # Timer
        self.timer = self.create_timer(0.2, self.timer_callback)  # 5 Hz

    def timer_callback(self):
        all_collected = True

        for i in range(1, self.num_strings + 1):
            start_frame = f'string{i}1'
            end_frame = f'string{i}2'

            for frame in [start_frame, end_frame]:
                try:
                    tf = self.tf_buffer.lookup_transform('world', frame, rclpy.time.Time())
                    trans = tf.transform.translation
                    pos = [trans.x, trans.y, trans.z]
                    if len(self.readings[frame]) < self.max_samples:
                        self.readings[frame].append(pos)
                        self.get_logger().info(f"Collected {len(self.readings[frame])} for {frame}")
                except Exception as e:
                    self.get_logger().warn(f"Transform for {frame} not available yet: {e}")
                    all_collected = False

        # Check if all samples are collected
        if all(len(self.readings[f'string{i}1']) >= self.max_samples and
               len(self.readings[f'string{i}2']) >= self.max_samples
               for i in range(1, self.num_strings + 1)):
            self.save_to_yaml()
            self.timer.cancel()

    def average(self, positions):
        n = len(positions)
        return [sum(p[i] for p in positions) / n for i in range(3)]

    def save_to_yaml(self):
        strings = []
        for i in range(1, self.num_strings + 1):
            start_avg = self.average(self.readings[f'string{i}1'])
            end_avg = self.average(self.readings[f'string{i}2'])
            strings.append({'start': start_avg, 'end': end_avg})

        config = {'strings': strings}

        output_path = '/home/benchmark/vicon_string_ws/src/measurement/string_positions.yaml'
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with open(output_path, 'w') as f:
            yaml.dump(config, f)

        self.get_logger().info(f"Saved averaged string positions to {output_path}")


def main(args=None):
    rclpy.init(args=args)
    node = StringTransformAverager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
