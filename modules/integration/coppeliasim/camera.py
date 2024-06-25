from modules.vision.camera import *

class CoppeliaSim_Camera(Camera): 
    def __init__(self, 
                 # Simulation handling
                 vision_sensor_handle,
 
                 # Intrinsic Parameters
                 resolution, 
                 fov_degrees=None, # If not given, consider uncalibrated
 
                 # Camera Pose
                 pose=np.eye(4), 
 
                 # Lens Distortion Model
                 distortion_model=None,
                 distortion_coefficients=np.zeros(4),
 
                 # Image Noise Model
                 snr_dB=np.inf # No noise
                 ):
        
        # Coppelia's handle
        self.vision_sensor_handle = vision_sensor_handle
        
        Camera.__init__(self,
                        resolution,
                        fov_degrees,
                        pose,
                        distortion_model,
                        distortion_coefficients,
                        snr_dB)
        
    def get_image(self, api_method):
        # Get grayscale image buffer
        buffer, resolution = api_method(self.vision_sensor_handle, 1) # Set second argument to 1 for grayscale, 0 for RGB

        # Convert buffer into single channel image
        image_unflipped = np.frombuffer(buffer, dtype=np.uint8).reshape(resolution[1], resolution[0])

        # In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
        # This is consistent with the axes of vision sensors, pointing Z outwards, Y up
        image_gray = cv2.flip(image_unflipped, 0)

        # Use cv2.remap with the custom remapped coordinates
        if self.distortion_model is not None:
            image_distorted = cv2.remap(src=image_gray,
                                        dst=image_gray, 
                                        map1=self.map_u, 
                                        map2=self.map_v, 
                                        interpolation=cv2.INTER_NEAREST)
            
        else:
            image_distorted = image_gray # No distortion is applied
        
        # Add noise based on the desired SNR for the image
        simulated_image = add_noise(image_distorted, self.snr)

        return simulated_image