import cv2
import glob
import os
import multiprocessing as mp
from cv_bridge import CvBridge

from utils.exif_utils import exif_data, parse_exif_struct, calculate_rt_hz


BRIDGE = CvBridge()

def load_single_image(args):
    """
    Load a single image from a given path and return a ROS Image message
    and extracted EXIF data.

    Parameters
    ----------
    args : tuple
        A tuple containing the image path and the maximum width.

    Returns
    -------
    tuple
        A tuple containing the ROS Image message and the extracted EXIF data.
    """
    path, max_width = args
    img = cv2.imread(path)
    if img is None:
        return None, None

    if img.shape[1] > max_width:
        scale = max_width / img.shape[1]
        img = cv2.resize(img, None, fx=scale, fy=scale)

    ros_img = BRIDGE.cv2_to_imgmsg(img, encoding="bgr8")
    del img

    ex_raw = exif_data(path)
    ex = parse_exif_struct(ex_raw)

    return ros_img, ex


def preload_all(folder, max_width, real_time=False, workers=None):
    """
    Preload all images in a given folder and return a list of ROS 
    Image messages, a list of extracted EXIF data dictionaries and the 
    real-time sampling frequency 
    in Hz.

    Parameters
    ----------
    folder : str
        The path to the folder containing the images.
    max_width : int
        The maximum width to which the images will be resized.
    real_time : bool, optional
        If True, calculate the real-time sampling frequency in Hz from the 
        timestamps in the EXIF data. Otherwise, set the sampling frequency 
        to 0.
    workers : int, optional
        The number of worker processes to use for preloading the images. 
        If None, use the number of available CPU cores.

    Returns
    -------
    tuple
        A tuple containing the list of ROS Image messages, the list of 
        extracted EXIF data dictionaries and the real-time sampling frequency 
        in Hz.
    """
    image_paths = sorted(glob.glob(os.path.join(folder, "*.JPG")))

    args_list = [(p, max_width) for p in image_paths]
    workers = workers or mp.cpu_count()

    ros_images, exif_list = [], []
    with mp.Pool(processes=workers) as pool:
        for ros_img, ex in pool.imap(load_single_image, args_list):
            if ros_img is not None:
                ros_images.append(ros_img)
                exif_list.append(ex)

    hz = calculate_rt_hz(
        [d["timestamp"] for d in exif_list]
    ) if real_time else 0.0

    return ros_images, exif_list, hz
