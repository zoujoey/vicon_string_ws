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

        # Number of horizontal strings (i.e., string11/string12 → stringn1/stringn2)
        self.num_strings = 1  # Adjust this as needed
        
        # Number of vertical strings (i.e., vstring11 → vstringn1)
        self.num_vertical_strings = 0  # Adjust this as needed
        
        # Vertical string height (±1m from marker position)
        self.vertical_string_height = 1.0
        
        # List of horizontal string indices whose endpoints should also be treated as vertical strings
        # Format: [(string_index, endpoint), ...] where endpoint is either 1 or 2
        # Example: [(1, 1), (1, 2)] means string11 and string12 are also vertical strings
        self.horizontal_as_vertical = [(1,1),(1,2)]  # Add tuples like (1, 1) or (2, 2) as needed

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

        # Collect horizontal strings
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

        # Collect vertical strings
        for i in range(1, self.num_vertical_strings + 1):
            vstring_frame = f'vstring{i}1'
            
            try:
                tf = self.tf_buffer.lookup_transform('world', vstring_frame, rclpy.time.Time())
                trans = tf.transform.translation
                pos = [trans.x, trans.y, trans.z]
                if len(self.readings[vstring_frame]) < self.max_samples:
                    self.readings[vstring_frame].append(pos)
                    self.get_logger().info(f"Collected {len(self.readings[vstring_frame])} for {vstring_frame}")
            except Exception as e:
                self.get_logger().warn(f"Transform for {vstring_frame} not available yet: {e}")
                all_collected = False

        # Check if all samples are collected
        horizontal_complete = all(len(self.readings[f'string{i}1']) >= self.max_samples and
                                 len(self.readings[f'string{i}2']) >= self.max_samples
                                 for i in range(1, self.num_strings + 1))
        
        vertical_complete = all(len(self.readings[f'vstring{i}1']) >= self.max_samples
                               for i in range(1, self.num_vertical_strings + 1))
        
        if horizontal_complete and vertical_complete:
            self.save_to_yaml()
            self.timer.cancel()

    def average(self, positions):
        n = len(positions)
        return [sum(p[i] for p in positions) / n for i in range(3)]

    def save_to_yaml(self):
        strings = []
        vertical_strings = []
        
        # Process horizontal strings
        for i in range(1, self.num_strings + 1):
            start_avg = self.average(self.readings[f'string{i}1'])
            end_avg = self.average(self.readings[f'string{i}2'])
            strings.append({'start': start_avg, 'end': end_avg})

        # Process vertical strings (dedicated vstring markers)
        for i in range(1, self.num_vertical_strings + 1):
            marker_avg = self.average(self.readings[f'vstring{i}1'])
            # Create start and end positions by adding/subtracting height
            start_z = marker_avg[2] - self.vertical_string_height
            end_z = marker_avg[2] + self.vertical_string_height
            
            # Ensure the bottom of the vertical string doesn't go below z=0
            if start_z < 0:
                start_z = 0.0
            
            start_pos = [marker_avg[0], marker_avg[1], start_z]
            end_pos = [marker_avg[0], marker_avg[1], end_z]
            vertical_strings.append({'start': start_pos, 'end': end_pos})

        # Process horizontal endpoints that should also be vertical strings
        for string_idx, endpoint in self.horizontal_as_vertical:
            frame_name = f'string{string_idx}{endpoint}'
            if frame_name in self.readings and len(self.readings[frame_name]) >= self.max_samples:
                marker_avg = self.average(self.readings[frame_name])
                # Create start and end positions by adding/subtracting height
                start_z = marker_avg[2] - self.vertical_string_height
                end_z = marker_avg[2] + self.vertical_string_height
                
                # Ensure the bottom of the vertical string doesn't go below z=0
                if start_z < 0:
                    start_z = 0.0
                
                start_pos = [marker_avg[0], marker_avg[1], start_z]
                end_pos = [marker_avg[0], marker_avg[1], end_z]
                vertical_strings.append({'start': start_pos, 'end': end_pos})

        config = {'strings': strings, 'vertical_strings': vertical_strings}

        output_path = '/home/benchmark/vicon_string_ws/src/measurement/string_positions.yaml'
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with open(output_path, 'w') as f:
            yaml.dump(config, f)

        self.get_logger().info(f"Saved averaged string positions to {output_path}")
        self.get_logger().info(f"Saved {len(strings)} horizontal strings and {len(vertical_strings)} vertical strings")
        if self.horizontal_as_vertical:
            self.get_logger().info(f"Included {len(self.horizontal_as_vertical)} horizontal endpoints as vertical strings")


def main(args=None):
    rclpy.init(args=args)
    node = StringTransformAverager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
