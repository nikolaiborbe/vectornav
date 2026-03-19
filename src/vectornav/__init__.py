"""VectorNav VN-200 Python Driver.

Cross-platform (Windows/macOS/Linux) driver for the VN-200 GPS/INS sensor
using the serial ASCII protocol over USB.

Usage:
    from vectornav import VN200

    with VN200("/dev/cu.usbserial-XXXX") as vn:
        print(vn.read_yaw())
        print(vn.read_accel_x())
"""

from vectornav._vn200 import VN200, VN200Error

__all__ = ["VN200", "VN200Error"]
__version__ = "0.1.0"
