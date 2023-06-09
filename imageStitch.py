import os
from PIL import Image
# note: current python version is 3.7.5 

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
file_names = sorted(os.listdir(image_directory), key=lambda x: int(x.split(".")[0][3:]))
panoImages = [os.path.join(image_directory, filename) for filename in file_names]

merged_image = merge_images(panoImages)
merged_image.show()
