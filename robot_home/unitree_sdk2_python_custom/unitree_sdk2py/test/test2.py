import pyrealsense2 as rs
import numpy as np
import cv2
import os
import time

class PixelToCoordinateConverter:
    def __init__(self):
        ctx = rs.context()
        devices = ctx.query_devices()
        for dev in devices:
           print(f"Device: {dev.get_info(rs.camera_info.name)}")
           print(f"Serial: {dev.get_info(rs.camera_info.serial_number)}")
    # Попробуем получить информацию о занятости
           try:
         # Это покажет, открыт ли device менеджером
               print("Device info accessible")
           except RuntimeError as e:
                print(f"Device busy: {e}")
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        self.intrinsics = None   

    def start(self):
        
        profile = self.pipeline.start(self.config)
        
     
        depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        self.intrinsics = depth_profile.get_intrinsics()
        
        os.makedirs('data/image', exist_ok=True)
        os.makedirs('data/color', exist_ok=True)
        os.makedirs('data/depth', exist_ok=True)

    def stop(self):
        self.pipeline.stop()

    def get_single_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        
        
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        return color_frame, depth_frame, color_image, depth_image

    def process_pixels(self, number):
        color_frame, depth_frame, color_image, depth_image = self.get_single_frame()
        
   
        cv2.imwrite(f"data/image/current_frame_{number}.png", color_image)
        #cv2.imwrite(f'data/color/color_b_{number}.png', color_image[:, :, 0])
        #cv2.imwrite(f'data/color/color_g_{number}.png', color_image[:, :, 1])
        #cv2.imwrite(f'data/color/color_r_{number}.png', color_image[:, :, 2])
        cv2.imwrite(f'data/depth/depth_{number}.png', depth_image)
        
        

    def capture_multiple_frames(self, num_frames=10, delay=3):
        """Capture and save multiple frames with a small delay between them."""
        for i in range(num_frames):
            print(f"Capturing frame {i+1}/{num_frames}")
            self.process_pixels(i)
            time.sleep(delay)   # respect the camera's frame rate


def main():
    converter = PixelToCoordinateConverter()
    time.sleep(1)
    try:
        converter.start()
        converter.capture_multiple_frames()   # capture 100 frames
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        converter.stop()
        print("Camera stopped. Program finished.")


if __name__ == "__main__":
    main()
