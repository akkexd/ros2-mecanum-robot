# SLAM Parameters Documentation

## Optimized Parameters for ROSMASTER X3

### Robot Specifications
- LiDAR: RPLIDAR A2M8 (12m range, 10Hz)
- Camera: Intel RealSense D435 (640x480, 30Hz)
- Wheel Odometry: Mecanum wheels with encoders

### Week 7 (Basic) vs Week 8 (High-Quality) Settings

| Parameter | Week 7 | Week 8 | Reason |
|-----------|--------|--------|--------|
| resolution | 0.05 | 0.03 | Finer detail |
| max_laser_range | 12.0 | 8.0 | More reliable |
| minimum_travel_distance | 0.3 | 0.2 | More scans |
| minimum_travel_heading | 0.3 | 0.2 | More scans |
| loop_match_minimum_chain_size | 10 | 15 | Conservative |
| scan_buffer_size | 10 | 15 | Better matching |

### Mapping Tips
1. Drive slowly (0.1-0.15 m/s for high-quality)
2. Always close loops (return to start)
3. Cover walls first, then interior
4. Multiple passes improve quality
5. Use camera to verify robot sees correctly

### slam_toolbox Lifecycle Note
Always use IncludeLaunchDescription with the built-in online_async_launch.py
to ensure proper lifecycle node initialization.

### Tested Environments
- Living room: 5m x 4m - Good results at 0.03 resolution
- Hallway: 10m x 2m - Needs multiple loop closures
