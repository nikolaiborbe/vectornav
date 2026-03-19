from vectornav import VN200
from time import sleep

with VN200("/dev/cu.usbserial-AU04Q4TG") as vn:
    print(vn.read_accel_x())        # m/s^2
    print(vn.read_yaw())            # degrees
    print(vn.read_gps_latitude())   # degrees
    print(vn.read_ins())            # full fused INS solution
    while True:
        print(vn.read_accel_x())    # GPS time in seconds since the GPS epoch
        sleep(0.1)                   # sleep for 100 ms

