#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import time
import numpy as np
import matplotlib.pyplot as plt
class LivoxProcessor(Node):
    def __init__(self):
        super().__init__('livox_processor')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud_livox_mid360',
            self.pointcloud_callback,
            10)
        self.cont=[]
        self.start_time = time.time()
        
    def pointcloud_callback(self, msg):
        
        if msg.point_step == 22:  # XYZI format (4 floats)
            try:
                points = np.frombuffer(msg.data, dtype=np.float16).reshape(-1, 4)
                elapsed = time.time() - self.start_time
              	   
                # Show first 3 points as example
                self.cont.append(np.array(points))
            
                
                # Show basic stats
                if len(points) > 0:
                    avg_x = np.mean(points[:, 0])
                    avg_y = np.mean(points[:, 1]) 
                    avg_z = np.mean(points[:, 2])
              
                    
            except Exception as e:
                print(f"Error parsing data: {e}")
        else:
            print(f"Unexpected format: {msg.point_step} bytes per point")
            print(f"   Try with: points = np.frombuffer(msg.data, dtype=np.float32)")
            print(f"   Then reshape: points.reshape(-1, {msg.point_step//4})")

def main(args=None):
    rclpy.init(args=args)
    node = LivoxProcessor()
    
 
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        
        print("Manual interrupt detected with Ctrl+C")
        
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
        for i in range(10):
         x=node.cont[i][:, 0]
         y=node.cont[i][:, 1]
         z=node.cont[i][:, 2]
        
         intensity=node.cont[i][:, 3]
        
         resolution = 1  # meters per pixel
        
         x=x[~np.isnan(x)]
         x=x[~np.isinf(x)]
        
        
         y=y[~np.isnan(y)]
         y=y[~np.isinf(y)]
         
        
         z=z[~np.isnan(z)]
         z=z[~np.isinf(z)]
         z=z[z<1e4]
         z=z[z>-1e4]
        
        
         a=x.shape[0]
         b=y.shape[0]
         if a>b:
           x=x[:b]
         else:
           y=y[:a]
         plt.scatter(x, y)
        plt.show()
if __name__ == '__main__':
    main()
