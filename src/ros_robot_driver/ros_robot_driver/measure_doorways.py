#!/usr/bin/env python3
"""
measure_doorways.py — Interactive Doorway Coordinate Measurement Tool

USAGE:
  1. Launch navigation with your map:
       ros2 launch ros_robot_bringup navigation.launch.py
  
  2. Run this tool:
       python3 measure_doorways.py
  
  3. In RViz, click "Publish Point" tool (top toolbar, looks like a green dot)
  
  4. Follow the prompts — click 3 points per doorway:
     - Doorway center (middle of the opening)
     - Staging point (0.6m before doorway, room side)
     - Exit point (0.6m after doorway, other side)
  
  5. The tool outputs a ready-to-paste YAML snippet

Place at: ~/ros2_ws/src/ros_robot_driver/ros_robot_driver/measure_doorways.py
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class DoorwayMeasurer(Node):
    def __init__(self):
        super().__init__('doorway_measurer')
        
        self.create_subscription(
            PointStamped, '/clicked_point', self.point_callback, 10
        )
        
        self.doorways = []
        self.current_doorway = {}
        self.step = 0
        self.steps = ['doorway_center', 'staging_point', 'exit_point']
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('DOORWAY MEASUREMENT TOOL')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
        self.get_logger().info('In RViz: select "Publish Point" tool')
        self.get_logger().info('Then click the map to mark points.')
        self.get_logger().info('')
        self.prompt_next()
    
    def prompt_next(self):
        if self.step < len(self.steps):
            point_name = self.steps[self.step]
            descriptions = {
                'doorway_center': 'CENTER of the doorway opening',
                'staging_point': '0.6m BEFORE the doorway (approach side)',
                'exit_point': '0.6m AFTER the doorway (exit side)',
            }
            self.get_logger().info(
                f'Click #{self.step + 1}: {descriptions[point_name]}'
            )
        else:
            self.finish_doorway()
    
    def point_callback(self, msg):
        x = msg.point.x
        y = msg.point.y
        point_name = self.steps[self.step]
        
        self.current_doorway[point_name] = (round(x, 3), round(y, 3))
        self.get_logger().info(
            f'  → {point_name}: ({x:.3f}, {y:.3f})'
        )
        
        self.step += 1
        self.prompt_next()
    
    def finish_doorway(self):
        # Compute transit heading from staging → exit
        s = self.current_doorway['staging_point']
        e = self.current_doorway['exit_point']
        heading = math.atan2(e[1] - s[1], e[0] - s[0])
        
        c = self.current_doorway['doorway_center']
        
        doorway_data = {
            'center': c,
            'staging': s,
            'exit': e,
            'heading': round(heading, 4),
        }
        self.doorways.append(doorway_data)
        
        n = len(self.doorways)
        self.get_logger().info('')
        self.get_logger().info(f'Doorway {n} captured!')
        self.get_logger().info(
            f'  Transit heading: {math.degrees(heading):.1f}° '
            f'({heading:.4f} rad)'
        )
        self.get_logger().info('')
        
        # Print YAML snippet
        self.get_logger().info('--- YAML SNIPPET ---')
        self.get_logger().info(f'  doorway_{n}:')
        self.get_logger().info(f'    doorway_center: [{c[0]}, {c[1]}]')
        self.get_logger().info(f'    zone_radius: 0.8')
        self.get_logger().info(f'    transit_heading: {heading:.4f}')
        self.get_logger().info(f'    staging_point: [{s[0]}, {s[1]}]')
        self.get_logger().info(f'    exit_point: [{e[0]}, {e[1]}]')
        self.get_logger().info(f'    width_m: 0.76')
        self.get_logger().info('--- END SNIPPET ---')
        self.get_logger().info('')
        self.get_logger().info(
            'Press Ctrl+C to finish, or click 3 more points '
            'for another doorway.'
        )
        self.get_logger().info('')
        
        # Reset for next doorway
        self.current_doorway = {}
        self.step = 0
        self.prompt_next()


def main(args=None):
    rclpy.init(args=args)
    node = DoorwayMeasurer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('')
        node.get_logger().info(f'Captured {len(node.doorways)} doorway(s).')
        node.get_logger().info('Copy the YAML snippets into doorway_config.yaml')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()