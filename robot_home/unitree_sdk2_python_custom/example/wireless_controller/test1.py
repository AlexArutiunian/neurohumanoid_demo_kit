import time
import cv2
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
from unitree_sdk2py.idl.default import unitree_go_msg_dds__VideoMessage_
from unitree_sdk2py.utils.image import bgr_to_rgb, rgb_to_bgr

class VideoServer:
    def __init__(self, camera_source=0, network_interface="enp2s0"):
        """
        Initialize video server
        :param camera_source: Camera source (0 for default camera, or video file path)
        :param network_interface: Network interface name
        """
        # Initialize channel factory
        ChannelFactoryInitialize(0, network_interface)
        
        # Create publisher for video stream
        self.publisher = ChannelPublisher("video", unitree_go_msg_dds__VideoMessage_)
        self.publisher.Init()
        
        # Initialize camera
        self.camera_source = camera_source
        self.cap = None
        self.init_camera()
        
        # Image parameters
        self.frame_width = 640
        self.frame_height = 480
        self.frame_rate = 30
        
        # Server state
        self.running = False
        
    def init_camera(self):
        """Initialize camera or video source"""
        try:
            self.cap = cv2.VideoCapture(self.camera_source)
            if not self.cap.isOpened():
                print(f"Error: Could not open camera source {self.camera_source}")
                # Try default camera
                self.cap = cv2.VideoCapture(0)
                if not self.cap.isOpened():
                    print("Error: Could not open default camera")
                    return False
            
            # Set camera properties
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
            
            return True
        except Exception as e:
            print(f"Error initializing camera: {e}")
            return False
    
    def capture_frame(self):
        """Capture a frame from camera"""
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                # Convert BGR to RGB (OpenCV uses BGR, but might need RGB for display/transmission)
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                return True, frame_rgb
        return False, None
    
    def create_video_message(self, image_data):
        """Create VideoMessage from image data"""
        msg = unitree_go_msg_dds__VideoMessage_()
        
        # Set timestamp
        current_time = time.time()
        msg.timestamp_sec = int(current_time)
        msg.timestamp_nanosec = int((current_time - int(current_time)) * 1e9)
        
        # Set image data
        if image_data is not None:
            # Flatten image to bytes
            height, width, channels = image_data.shape
            msg.data = image_data.tobytes()
            msg.size = len(msg.data)
            msg.width = width
            msg.height = height
            msg.channel = channels
            msg.frame_type = 1  # Typically 1 for RGB images
            
        return msg
    
    def send_image_sample(self):
        """Capture and send a single image sample"""
        try:
            # Capture frame
            success, frame = self.capture_frame()
            if not success:
                print("Failed to capture frame")
                return -1
            
            # Create message
            msg = self.create_video_message(frame)
            
            # Publish message
            self.publisher.Write(msg)
            print(f"Image sent: {msg.width}x{msg.height}, size: {msg.size} bytes")
            return 0
            
        except Exception as e:
            print(f"Error sending image: {e}")
            return -1
    
    def start_streaming(self, interval=0.033):
        """
        Start continuous video streaming
        :param interval: Time between frames in seconds (1/fps)
        """
        self.running = True
        print(f"Starting video streaming at {1/interval:.1f} fps...")
        
        try:
            while self.running:
                start_time = time.time()
                
                # Send current frame
                self.send_image_sample()
                
                # Calculate sleep time to maintain frame rate
                elapsed = time.time() - start_time
                sleep_time = max(0, interval - elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print("\nStopping video stream...")
        finally:
            self.stop()
    
    def stop(self):
        """Stop the server and release resources"""
        self.running = False
        if self.cap:
            self.cap.release()
        print("Video server stopped")
    
    def set_resolution(self, width, height):
        """Set camera resolution"""
        self.frame_width = width
        self.frame_height = height
        if self.cap:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    
    def set_frame_rate(self, fps):
        """Set frame rate"""
        self.frame_rate = fps
        if self.cap:
            self.cap.set(cv2.CAP_PROP_FPS, fps)


# Alternative: Simple server that responds to requests
class RequestVideoServer:
    def __init__(self, camera_source=0, network_interface="enp2s0"):
        """Server that responds to GetImageSample requests"""
        from unitree_sdk2py.core.channel import ChannelService
        
        ChannelFactoryInitialize(0, network_interface)
        
        # Create service server
        self.service = ChannelService("video_service", unitree_go_msg_dds__VideoMessage_)
        self.service.Init(self.handle_request)
        
        # Initialize camera
        self.cap = cv2.VideoCapture(camera_source)
        if not self.cap.isOpened():
            print("Warning: Could not open camera, using test pattern")
            self.cap = None
        
        print("Video service server started, waiting for requests...")
    
    def handle_request(self, request):
        """Handle incoming image requests"""
        # This function is called when client requests an image
        try:
            if self.cap and self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret:
                    # Convert to RGB
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    
                    # Create response
                    response = unitree_go_msg_dds__VideoMessage_()
                    current_time = time.time()
                    response.timestamp_sec = int(current_time)
                    response.timestamp_nanosec = int((current_time - int(current_time)) * 1e9)
                    response.data = frame_rgb.tobytes()
                    response.size = len(response.data)
                    response.width = frame.shape[1]
                    response.height = frame.shape[0]
                    response.channel = 3
                    response.frame_type = 1
                    
                    return 0, response
            else:
                # Create a test pattern if no camera available
                return self.create_test_pattern()
                
        except Exception as e:
            print(f"Error handling request: {e}")
            return -1, unitree_go_msg_dds__VideoMessage_()
    
    def create_test_pattern(self):
        """Create a test pattern image"""
        # Create a simple color pattern
        height, width = 480, 640
        channels = 3
        
        # Create gradient pattern
        frame = np.zeros((height, width, channels), dtype=np.uint8)
        for y in range(height):
            for x in range(width):
                frame[y, x] = [
                    int(x * 255 / width),      # R
                    int(y * 255 / height),     # G
                    int((x + y) * 255 / (width + height))  # B
                ]
        
        # Create response
        response = unitree_go_msg_dds__VideoMessage_()
        current_time = time.time()
        response.timestamp_sec = int(current_time)
        response.timestamp_nanosec = int((current_time - int(current_time)) * 1e9)
        response.data = frame.tobytes()
        response.size = len(response.data)
        response.width = width
        response.height = height
        response.channel = channels
        response.frame_type = 1
        
        return 0, response
    
    def run(self):
        """Run the server indefinitely"""
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nShutting down server...")
            if self.cap:
                self.cap.release()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Unitree Go2 Video Server')
    parser.add_argument('--mode', choices=['stream', 'request'], default='stream',
                       help='Server mode: stream (continuous) or request (on-demand)')
    parser.add_argument('--camera', type=int, default=0,
                       help='Camera source index')
    parser.add_argument('--interface', type=str, default="enp2s0",
                       help='Network interface name')
    parser.add_argument('--fps', type=int, default=30,
                       help='Frame rate for streaming mode')
    parser.add_argument('--width', type=int, default=640,
                       help='Image width')
    parser.add_argument('--height', type=int, default=480,
                       help='Image height')
    
    args = parser.parse_args()
    
    if args.mode == 'stream':
        server = VideoServer(
            camera_source=args.camera,
            network_interface=args.interface
        )
        server.set_resolution(args.width, args.height)
        server.set_frame_rate(args.fps)
        server.start_streaming(interval=1.0/args.fps)
    else:
        server = RequestVideoServer(
            camera_source=args.camera,
            network_interface=args.interface
        )
        server.run()
