import subprocess
from datetime import datetime

def exif_data(image_path):
    """
    Extract EXIF data from a given image file.

    Parameters
    ----------
    image_path : str
        The path to the image file.

    Returns
    -------
    dict
        A dictionary containing the extracted EXIF data.
    """
    result = subprocess.run(
        ['exiftool', image_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    if result.returncode != 0:
        print("EXIF ERROR:", result.stderr)
        return None

    ex = {}
    for line in result.stdout.split("\n"):
        if " : " in line:
            key, value = line.split(" : ", 1)
            ex[key.strip()] = value.strip()

    return ex

def parse_exif_struct(ex):
    """
    Parse an EXIF data dictionary into a structured dictionary.

    Parameters
    ----------
    ex : dict
        A dictionary containing the extracted EXIF data.

    Returns
    -------
    dict
        A structured dictionary containing the extracted EXIF data.
    """
    return {
        "timestamp": datetime.strptime(
            str(ex["Date/Time Original"]),
            "%Y:%m:%d %H:%M:%S.%f"
        ),
        "latitude": float(ex["GPS Latitude Raw"]),
        "longitude": float(ex["GPS Longitude Raw"]),
        "altitude": float(ex["GPS Altitude Raw"]),
        "vehicle_yaw": float(ex["Vehicle Orientation NED Yaw"])
    }


def calculate_rt_hz(timestamps):
    """
    Calculate the real-time sampling frequency in Hz from a list of 
    timestamps.

    Parameters
    ----------
    timestamps : list of datetime
        A list of timestamps.

    Returns
    -------
    float
        The real-time sampling frequency in Hz.
    """
    deltas = [
        (t2 - t1).total_seconds()
        for t1, t2 in zip(timestamps[:-1], timestamps[1:])
    ]
    avg = sum(deltas) / len(deltas)
    
    return 1.0 / avg if avg > 0 else 30.0
