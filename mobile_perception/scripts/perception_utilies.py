import ament_index_python.packages
import os


def get_package_path(package_name):
    try:
        return ament_index_python.packages.get_package_share_directory(
            package_name)
    except ament_index_python.packages.PackageNotFoundError:
        print(f"Package '{package_name}' not found.")
        return None
