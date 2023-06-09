import os
from PIL import Image

def merge_images(file_paths):
    """
    Merges images horizontally, in the order they are passed in.

    :param file_paths: list of file paths to images
    :return: merged image
    """
    images = [Image.open(file) for file in file_paths]

    widths, heights = zip(*(image.size for image in images))

    result_width = sum(widths)
    result_height = max(heights)

    result = Image.new('RGB', (result_width, result_height))

    x_offset = 0
    for image in images:
        result.paste(image, (x_offset, 0))
        x_offset += image.width

    return result

image_directory = "imageStitchTest"
file_names = sorted(os.listdir(image_directory), key=lambda x: int(x.split(".")[0][6:]))
panoImages = [os.path.join(image_directory, filename) for filename in file_names]

merged_image = merge_images(panoImages)
merged_image.show()

# Create the 'Pano' directory if it doesn't exist
output_directory = "Pano"
os.makedirs(output_directory, exist_ok=True)

# Find the highest existing count in the 'Pano' directory
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

# Increment the count and construct the output file name
count = highest_count + 1
output_filename = f"{count:03d}_merged_image.jpg"
output_path = os.path.join(output_directory, output_filename)

# Save the merged image
merged_image.save(output_path)
