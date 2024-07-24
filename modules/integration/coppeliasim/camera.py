from modules.vision.camera import *

class CoppeliaSim_Camera(Camera): 
    def __init__(self, 
                 # Simulation handling
                 vision_sensor_handle=None,
 
                 # Intrinsic Parameters
                 resolution=(256, 256), # Standard Vision Sensor resolution
                 fov_degrees=60, # Standard Vision Sensor FOV in degrees
 
                 # Extrinsic Parameters
                 pose=np.eye(4), # Aligned with the world's reference frame
 
                 # Lens Distortion Model
                 distortion_model=None,
                 distortion_coefficients=np.zeros(4),
 
                 # Image Noise Model
                 snr_dB=np.inf # No noise
                 ):
        
        # CoppeliaSim's handle
        self.vision_sensor_handle = vision_sensor_handle

        # Generating Matrices
        intrinsic_matrix = build_intrinsic_matrix(fov_degrees=fov_degrees,
                                                  resolution=resolution)
        extrinsic_matrix = np.linalg.inv(pose)

        # CoppeliaSim's Vision Sensor parameters
        self.fov_degrees = fov_degrees
        self.fov_radians = np.radians(self.fov_degrees)
        
        Camera.__init__(self,
                        resolution=resolution, 
                 
                        # Pinhole Camera Model
                        intrinsic_matrix=intrinsic_matrix,
                        extrinsic_matrix=extrinsic_matrix, 
        
                        # Lens Distortion Model
                        distortion_model=distortion_model, 
                        distortion_coefficients=distortion_coefficients, 
        
                        # Image Noise Model
                        snr_dB=snr_dB)
        
    def get_image(self, api_method):
        # If any Vision Sensor handle is associated with camera, return black image
        if self.vision_sensor_handle is None:
            return np.zeros(self.resolution)

        # Get grayscale image buffer
        buffer, resolution = api_method(self.vision_sensor_handle, 1) # Set second argument to 1 for grayscale, 0 for RGB

        # Convert buffer into single channel image
        image_unflipped = np.frombuffer(buffer, dtype=np.uint8).reshape(resolution[1], resolution[0])

        # In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
        # This is consistent with the axes of vision sensors, pointing Z outwards, Y up
        image_noiseless = cv2.flip(image_unflipped, 0)

        # Use cv2.remap with the custom remapped coordinates
        if self.distortion_model is not None:
            image_distorted = self.distort_image(image_noiseless)
            
        else:
            image_distorted = image_noiseless # No distortion is applied
        
        # Add noise based on the desired SNR for the image
        modeled_image = self.noise_image(image_distorted)

        return modeled_image