#!/usr/bin/env python3
"""
rosbag2 format utilities
"""
import os
from pathlib import Path

from rosbags.convert.converter import convert_2to1


def is_rosbag2(bag_path: str):
    """
    Check if a file is a rosbag2 file.
    :param path: path to the file

    :return: True if the file is a rosbag2 file, False otherwise
    """

    if os.path.isdir(bag_path):
        metadata_file = os.path.join(bag_path, 'metadata.yaml')
        if os.path.exists(metadata_file):
            return True

    return False


def convert_rosbag2_to_rosbag1(bag_path: str):
    """
    Convert a rosbag2 file to a rosbag1 file.
    :param path: path to the file

    :return: path to the converted file or empty string if the conversion failed
    """

    src_path = Path(bag_path)
    dst_path = Path(bag_path).with_suffix('.bag')
    if dst_path.exists():
        print(f"Destination file already exists: {dst_path.as_posix()}. Aborting conversion.")
    else:
        try:
            convert_2to1(src_path, dst_path, [], [])
        except Exception as e:
            print(e)
            return ""

    return dst_path.as_posix()
