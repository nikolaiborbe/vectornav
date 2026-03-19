# VectorNav VN-200T Driver

## Project overview
Python driver for the VectorNav VN-200T GPS/INS sensor using pyserial and the ASCII protocol.
Target: cross-platform (Windows, macOS, Linux) via USB serial.

## Architecture
- pip package `vectornav-vn200`. Install with `pip install -e .` or `pip install .`.
- Source layout: `src/vectornav/` with `__init__.py` (exports VN200, VN200Error) and `_vn200.py` (driver).
- `pyproject.toml` defines the build (setuptools). Dependencies: `pyserial>=3.5`.
- Uses ASCII protocol: `$VNRRG,<reg_id>*<checksum>\r\n`. No binary protocol.
- Async outputs are paused on connect, resumed on disconnect.
- 8-bit XOR checksum (computed, not bypassed).

## Key register map
| Register | Name | Data |
|----------|------|------|
| 1 | Model Number | Product name string |
| 3 | Serial Number | uint32 |
| 4 | Firmware Version | Version string |
| 8 | YPR | Yaw/Pitch/Roll (deg) |
| 9 | Quaternion | q0-q3 |
| 15 | Quat+Mag+Accel+Gyro | Quaternion + compensated sensors |
| 17 | Magnetic | Compensated mag XYZ (Gauss) |
| 18 | Acceleration | Compensated accel XYZ (m/s^2) |
| 19 | Angular Rate | Compensated gyro XYZ (rad/s) |
| 20 | Mag+Accel+Gyro | All compensated sensors combined |
| 27 | YPR+Mag+Accel+Gyro | Attitude + all compensated sensors |
| 54 | IMU Measurements | Uncompensated mag/accel/gyro + temp + pressure |
| 58 | GPS LLA | GPS position/velocity in lat/lon/alt |
| 59 | GPS ECEF | GPS position/velocity in ECEF |
| 63 | INS LLA | INS fused solution in lat/lon/alt |
| 64 | INS ECEF | INS fused solution in ECEF |
| 72 | INS State LLA | Full INS state (attitude + pos + vel + accel + angular rate) |
| 73 | INS State ECEF | Full INS state in ECEF |
| 80 | Delta Theta/Vel | Coning/sculling integrated deltas |
| 239 | YPR+BodyAccel+Gyro | Attitude + true body accel (no gravity) + gyro |

## Conventions
- Datasheet: `VN200UserManual_UM004_080514.pdf` in project root.
- Default baud rate: 115200.
- All methods return dicts or floats. Keys use snake_case.
- Individual axis methods (e.g. `read_accel_x()`) call the batch method internally.
