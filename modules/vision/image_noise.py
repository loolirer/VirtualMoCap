import numpy as np

def add_noise(snr_dB, image, camera):
    snr_dB = 20 # Desired SNR value in db
    snr = np.power(10, (snr_dB / 20)) # Converted for ratio

    sorted_pixel_values = np.sort(image.ravel()).astype(np.float64) 
    sorted_pixels_coords = np.array(np.unravel_index(np.argsort(image.ravel()),camera.resolution)).T

    threshold = local_luminosity = 10 # Maximum luminosity difference of each set 
    i = j = 0 # Pointers

    for j, pixel_luminosity in enumerate(sorted_pixel_values):
        if i == j:
            continue # Skip if pointers are on the same place

        # Check if pixel surpasses threshold or the array has ended
        if pixel_luminosity > threshold or j == len(sorted_pixel_values) - 1:
            if j == len(sorted_pixel_values) - 1: # If last iteration
                j += 1 # Include last element

            mu = np.mean(sorted_pixel_values[i:j]) # Average value of the signal
            sigma = mu / snr # Standard deviation of the signal
            noise = np.random.normal(0, sigma, sorted_pixel_values[i:j].shape)
            sorted_pixel_values[i:j] += noise # Add noise

            i = j # Update pointers
            threshold = pixel_luminosity + local_luminosity # New threshold

    # Generate image with noised pixel values
    image_noisy = np.zeros(camera.resolution)
    for p, pixel_coord in enumerate(sorted_pixels_coords):
        u, v = pixel_coord
        image_noisy[u][v] = sorted_pixel_values[p]

    image_noisy[image_noisy < 0] = 0 # Clip negative pixel values to 0 
    image_noisy[image_noisy > 255] = 255 # Clip saturated pixel values to maxium 8-bit interger value 
    image_noisy = image_noisy.astype(np.uint8) # Cast as 8-bit interger 

    return image_noisy