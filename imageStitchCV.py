import cv2
import numpy as np
import os

def merge_images(file_paths):
    """
    Merge images horizontally, in the order they are passed in.

    :param file_paths: list of file paths to images
    :return: merged image
    """


    images = [cv2.imread(file) for file in file_paths]
    max_height = max(image.shape[0] for image in images)

    images = [cv2.resize(image, (image.shape[1] * max_height // image.shape[0], max_height)) for image in images]
    
    merged_image = cv2.hconcat(images)
    
    return merged_image

image_directory = "imageStitchTest"
file_names = sorted(os.listdir(image_directory), key=lambda x: int(x.split(".")[0][6:]))
panoImages = [os.path.join(image_directory, filename) for filename in file_names]

merged_image = merge_images(panoImages)

output_directory = "Pano"
os.makedirs(output_directory, exist_ok=True)

existing_files = os.listdir(output_directory)
existing_counts = []
for file_name in existing_files:
    if file_name.endswith(".jpg"):
        try:
            count = int(file_name.split("_")[0])
            existing_counts.append(count)
        except ValueError:
            pass

highest_count = max(existing_counts) if existing_counts else 0

count = highest_count + 1
output_filename = f"{count:03d}_merged_image.jpg"
output_path = os.path.join(output_directory, output_filename)

# Save the merged image
cv2.imwrite(output_path, merged_image)
