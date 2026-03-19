# vectornav-vn200

Cross-platform Python driver for the [VectorNav VN-200](https://www.vectornav.com/products/detail/vn-200) GPS/INS sensor over USB serial.

Uses the ASCII register protocol with `pyserial`. No vendor SDK required.

## Install

```bash
pip install vectornav-vn200
```

## Quick start

```python
from vectornav import VN200

with VN200("/dev/cu.usbserial-XXXX") as vn:  # or "COM3" on Windows
    print(vn.read_yaw())           # degrees
    print(vn.read_accel_x())       # m/s^2
    print(vn.read_gps_latitude())  # degrees (WGS84)
    print(vn.read_temperature())   # Celsius
```

### Find your serial port

```python
from vectornav import VN200
print(VN200.list_serial_ports())
```

Typical port names:

| OS      | Port                          |
|---------|-------------------------------|
| macOS   | `/dev/cu.usbserial-XXXXXXXX`  |
| Linux   | `/dev/ttyUSB0`                |
| Windows | `COM3`                        |

## API reference

### Connection

| Method | Description |
|--------|-------------|
| `VN200(port, baudrate=115200, timeout=1.0)` | Create a driver instance |
| `connect()` / `disconnect()` | Open/close serial. Also works as a context manager (`with`) |
| `list_serial_ports()` | Static method. Returns list of available serial port names |

### Device info

| Method | Returns | Unit |
|--------|---------|------|
| `read_model_number()` | `str` | - |
| `read_serial_number()` | `int` | - |
| `read_hardware_revision()` | `int` | - |
| `read_firmware_version()` | `str` | - |

### Attitude

| Method | Returns | Unit |
|--------|---------|------|
| `read_yaw()` | `float` | deg |
| `read_pitch()` | `float` | deg |
| `read_roll()` | `float` | deg |
| `read_yaw_pitch_roll()` | `dict` | deg |
| `read_quaternion()` | `dict` (q0-q3) | unitless |

### Magnetometer (compensated)

| Method | Returns | Unit |
|--------|---------|------|
| `read_mag_x()` | `float` | Gauss |
| `read_mag_y()` | `float` | Gauss |
| `read_mag_z()` | `float` | Gauss |
| `read_mag()` | `dict` | Gauss |

### Accelerometer (compensated, includes gravity)

| Method | Returns | Unit |
|--------|---------|------|
| `read_accel_x()` | `float` | m/s^2 |
| `read_accel_y()` | `float` | m/s^2 |
| `read_accel_z()` | `float` | m/s^2 |
| `read_accel()` | `dict` | m/s^2 |

### Body acceleration (gravity removed)

Reads ~0 when stationary. Uses the onboard attitude estimate to subtract gravity.

| Method | Returns | Unit |
|--------|---------|------|
| `read_body_accel_x()` | `float` | m/s^2 |
| `read_body_accel_y()` | `float` | m/s^2 |
| `read_body_accel_z()` | `float` | m/s^2 |

### Gyroscope (compensated)

| Method | Returns | Unit |
|--------|---------|------|
| `read_gyro_x()` | `float` | rad/s |
| `read_gyro_y()` | `float` | rad/s |
| `read_gyro_z()` | `float` | rad/s |
| `read_gyro()` | `dict` | rad/s |

### Temperature & pressure

| Method | Returns | Unit |
|--------|---------|------|
| `read_temperature()` | `float` | deg C |
| `read_pressure()` | `float` | kPa |

### GPS (5 Hz, onboard receiver)

| Method | Returns | Unit |
|--------|---------|------|
| `read_gps()` | `dict` | Full GPS solution (LLA + velocity + accuracy) |
| `read_gps_latitude()` | `float` | deg |
| `read_gps_longitude()` | `float` | deg |
| `read_gps_altitude()` | `float` | m (above WGS84 ellipsoid) |
| `read_gps_fix()` | `int` | 0=None, 1=Time, 2=2D, 3=3D |
| `read_gps_num_satellites()` | `int` | - |
| `read_gps_ned_velocity()` | `dict` | m/s (north/east/down) |
| `read_gps_ecef()` | `dict` | Full GPS solution in ECEF frame |

### INS (fused GPS + IMU, up to 400 Hz)

| Method | Returns | Unit |
|--------|---------|------|
| `read_ins()` | `dict` | INS solution (LLA + attitude + velocity + uncertainties) |
| `read_ins_latitude()` | `float` | deg |
| `read_ins_longitude()` | `float` | deg |
| `read_ins_altitude()` | `float` | m |
| `read_ins_ned_velocity()` | `dict` | m/s |
| `read_ins_ecef()` | `dict` | INS solution in ECEF frame |
| `read_ins_state_lla()` | `dict` | Full state: attitude + pos + vel + accel + angular rate |
| `read_ins_state_ecef()` | `dict` | Full state in ECEF |

### Batch reads (fewer serial round-trips)

| Method | Fields | Description |
|--------|--------|-------------|
| `read_mag_accel_gyro()` | 9 floats | All compensated 9-axis data |
| `read_ypr_mag_accel_gyro()` | 12 floats | Attitude + 9-axis |
| `read_ypr_body_accel_gyro()` | 9 floats | Attitude + body accel (no gravity) + gyro |
| `read_quat_mag_accel_gyro()` | 13 floats | Quaternion + 9-axis |
| `read_imu()` | 11 floats | Uncompensated 9-axis + temperature + pressure |
| `read_delta_theta_velocity()` | 7 floats | Coning/sculling integrated deltas |

### Helpers

| Method | Description |
|--------|-------------|
| `VN200.parse_ins_status(status)` | Decode INS status bitfield into mode, gps_fix, error |

## Coordinate frames

- **Body frame**: Right-handed, fixed to sensor. X forward, Y right, Z down (see datasheet Figure 5).
- **NED**: North-East-Down. Used for GPS/INS velocities.
- **ECEF**: Earth-Centered Earth-Fixed. Alternative position/velocity frame.
- **LLA**: Latitude/Longitude/Altitude on WGS84 ellipsoid.

## Requirements

- Python >= 3.10
- `pyserial >= 3.5`
- VN-200 connected via USB (FTDI serial adapter)
