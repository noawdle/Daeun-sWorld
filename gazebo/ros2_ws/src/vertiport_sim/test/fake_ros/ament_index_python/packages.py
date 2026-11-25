import os


def get_package_share_directory(name):
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
