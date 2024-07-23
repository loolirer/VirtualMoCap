# Importing modules...
import numpy as np

def add_noise(image_noiseless, snr):
    if snr == np.inf:
        return image_noiseless
    
    image_noisy = np.random.normal(image_noiseless.astype(np.float32), image_noiseless / snr)

    image_noisy = np.clip(0, 255, image_noisy).astype(np.uint8)

    return image_noisy
