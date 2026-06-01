#!/usr/bin/env python3
import time
import numpy as np
import cv2
import lcm
from dimos_lcm.sensor_msgs import Image
from dimos_lcm.nav_msgs import OccupancyGrid
from dimos_lcm.geometry_msgs import PoseStamped

def main():
    lc = lcm.LCM()
    print("Starting Perception Provider Simulation...")

    # Create a dummy costmap
    grid = [0] * (100 * 100)
    for i in range(20, 80):
        grid[i * 100 + 50] = 100 # Vertical wall
    
    while True:
        # 1. Publish Dummy Image
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(frame, f"DIMOS LIVE FEED - {time.time():.2f}", (50, 240), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        _, buffer = cv2.imencode('.jpg', frame)
        
        img_msg = Image()
        img_msg.width = 640
        img_msg.height = 480
        img_msg.data = buffer.tobytes()
        img_msg.size = len(img_msg.data)
        lc.publish("/color_image", img_msg.encode())

        # 2. Publish Dummy Costmap
        map_msg = OccupancyGrid()
        map_msg.width = 100
        map_msg.height = 100
        map_msg.resolution = 0.05
        map_msg.grid = grid
        lc.publish("/global_costmap", map_msg.encode())

        # 3. Publish Dummy Pose
        pose_msg = PoseStamped()
        pose_msg.position.x = 2.0 + np.sin(time.time()) * 0.5
        pose_msg.position.y = 1.0
        lc.publish("/odom", pose_msg.encode())

        time.sleep(0.1)

if __name__ == "__main__":
    main()
