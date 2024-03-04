import numpy as np

def add_noise(image_gray, snr):
    if snr == np.inf:
        return image_gray
    
    image_noisy = np.random.normal(image_gray.astype(np.float32), image_gray / snr)

    image_noisy = np.clip(0, 255, image_noisy).astype(np.uint8)

    return image_noisy
