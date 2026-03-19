"""
VectorNav VN-200 Python Driver

Cross-platform (Windows/macOS/Linux) driver for the VN-200 GPS/INS sensor
using the serial ASCII protocol over USB. Uses polling (register reads).

Reference: VN-200 User Manual UM004 v2.02

Usage:
    from vectornav import VN200

    with VN200("/dev/ttyUSB0") as vn:
        print(vn.read_yaw())
        print(vn.read_accel_x())
        print(vn.read_gps_latitude())
"""

import serial
import time
from typing import Optional


class VN200Error(Exception):
    """Base exception for VN-200 errors."""

    ERROR_CODES = {
        1: "Hard Fault",
        2: "Serial Buffer Overflow",
        3: "Invalid Checksum",
        4: "Invalid Command",
        5: "Not Enough Parameters",
        6: "Too Many Parameters",
        7: "Invalid Parameter",
        8: "Invalid Register",
        9: "Unauthorized Access",
        10: "Watchdog Reset",
        11: "Output Buffer Overflow",
        12: "Insufficient Baud Rate",
        255: "Error Buffer Overflow",
    }

    def __init__(self, code: int):
        self.code = code
        desc = self.ERROR_CODES.get(code, "Unknown Error")
        super().__init__(f"VN-200 error {code}: {desc}")


class VN200:
    """Driver for the VectorNav VN-200 GPS/INS sensor.

    Communicates over serial (USB) using the ASCII protocol.
    All methods poll registers on demand (no background streaming).

    Args:
        port: Serial port name. Examples:
            - Windows: "COM3"
            - macOS:   "/dev/tty.usbserial-XXXX"
            - Linux:   "/dev/ttyUSB0"
        baudrate: Serial baud rate. Factory default is 115200.
        timeout: Serial read timeout in seconds.
    """

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._ser: Optional[serial.Serial] = None

    # ── Connection ────────────────────────────────────────────────────

    def connect(self) -> None:
        """Open the serial connection and pause async outputs."""
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        time.sleep(0.1)
        self._ser.reset_input_buffer()
        # Pause asynchronous output so polling isn't contaminated.
        self._send_command("VNASY,0")

    def disconnect(self) -> None:
        """Resume async outputs and close the serial connection."""
        if self._ser and self._ser.is_open:
            try:
                self._send_command("VNASY,1")
            except Exception:
                pass
            self._ser.close()
        self._ser = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()

    # ── Low-level protocol ────────────────────────────────────────────

    @staticmethod
    def _checksum(data: str) -> str:
        """Compute 8-bit XOR checksum of the string between $ and *.

        Args:
            data: The payload between $ and * (exclusive).

        Returns:
            Two-character uppercase hex checksum string.
        """
        cksum = 0
        for ch in data:
            cksum ^= ord(ch)
        return f"{cksum:02X}"

    def _send_command(self, payload: str) -> str:
        """Send an ASCII command and return the response payload.

        Args:
            payload: Command without $ prefix or *checksum suffix.
                     Example: "VNRRG,8"

        Returns:
            The full response line (without $ and *checksum).

        Raises:
            VN200Error: If the sensor returns an error response.
            ConnectionError: If the serial port is not open.
            TimeoutError: If no valid response is received.
        """
        if not self._ser or not self._ser.is_open:
            raise ConnectionError("Serial port is not open. Call connect() first.")

        cs = self._checksum(payload)
        cmd = f"${payload}*{cs}\r\n"
        self._ser.write(cmd.encode("ascii"))

        # Read responses, skipping any stale async messages.
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            line = self._ser.readline().decode("ascii", errors="replace").strip()
            if not line:
                continue

            # Strip leading '$' if present.
            if line.startswith("$"):
                line = line[1:]

            # Strip checksum suffix.
            if "*" in line:
                line = line[: line.rfind("*")]

            # Check for error response.
            if line.startswith("VNERR,"):
                code = int(line.split(",")[1])
                raise VN200Error(code)

            # Accept response that matches our command type.
            cmd_prefix = payload.split(",")[0]
            if line.startswith(cmd_prefix):
                return line

        raise TimeoutError(f"No response for command: {payload}")

    def _read_register(self, reg_id: int) -> list[str]:
        """Read a register and return the parsed comma-separated fields.

        The first field (register ID echo) is stripped.

        Args:
            reg_id: The register number to read.

        Returns:
            List of string values from the response.
        """
        response = self._send_command(f"VNRRG,{reg_id}")
        parts = response.split(",")
        # parts[0] = "VNRRG", parts[1] = reg_id, parts[2:] = data fields
        return parts[2:]

    # ── Device info (registers 0-4) ──────────────────────────────────

    def read_model_number(self) -> str:
        """Read the product model name (e.g. "VN-200T-DEV").

        Register 1. Max 24 characters.
        """
        return self._read_register(1)[0]

    def read_hardware_revision(self) -> int:
        """Read the hardware revision number.

        Register 2.
        """
        return int(self._read_register(2)[0])

    def read_serial_number(self) -> int:
        """Read the device serial number (32-bit unsigned integer).

        Register 3.
        """
        return int(self._read_register(3)[0])

    def read_firmware_version(self) -> str:
        """Read the firmware version string (e.g. "1.0.0.0").

        Register 4.
        """
        return self._read_register(4)[0]

    # ── Attitude (registers 8, 9) ────────────────────────────────────

    def read_yaw_pitch_roll(self) -> dict:
        """Read attitude as yaw, pitch, and roll.

        Register 8. 3,2,1 Euler angles relative to NED frame.

        Returns:
            dict with keys: yaw, pitch, roll (all in degrees).
        """
        f = self._read_register(8)
        return {"yaw": float(f[0]), "pitch": float(f[1]), "roll": float(f[2])}

    def read_yaw(self) -> float:
        """Read yaw angle in degrees. Positive = right-handed rotation about Z."""
        return self.read_yaw_pitch_roll()["yaw"]

    def read_pitch(self) -> float:
        """Read pitch angle in degrees. Positive = right-handed rotation about Y."""
        return self.read_yaw_pitch_roll()["pitch"]

    def read_roll(self) -> float:
        """Read roll angle in degrees. Positive = right-handed rotation about X."""
        return self.read_yaw_pitch_roll()["roll"]

    def read_quaternion(self) -> dict:
        """Read attitude as a quaternion.

        Register 9. q = [q0, q1, q2, q3] where q3 is the scalar component.

        Returns:
            dict with keys: q0, q1, q2, q3 (unitless).
        """
        f = self._read_register(9)
        return {
            "q0": float(f[0]),
            "q1": float(f[1]),
            "q2": float(f[2]),
            "q3": float(f[3]),
        }

    # ── Compensated IMU individual sensors (registers 17, 18, 19) ────

    def read_mag(self) -> dict:
        """Read compensated magnetometer measurement.

        Register 17.

        Returns:
            dict with keys: mag_x, mag_y, mag_z (all in Gauss).
        """
        f = self._read_register(17)
        return {"mag_x": float(f[0]), "mag_y": float(f[1]), "mag_z": float(f[2])}

    def read_mag_x(self) -> float:
        """Read compensated magnetometer X-axis in Gauss."""
        return self.read_mag()["mag_x"]

    def read_mag_y(self) -> float:
        """Read compensated magnetometer Y-axis in Gauss."""
        return self.read_mag()["mag_y"]

    def read_mag_z(self) -> float:
        """Read compensated magnetometer Z-axis in Gauss."""
        return self.read_mag()["mag_z"]

    def read_accel(self) -> dict:
        """Read compensated accelerometer measurement.

        Register 18. Includes gravity when stationary (~9.8 m/s^2 on Z).

        Returns:
            dict with keys: accel_x, accel_y, accel_z (all in m/s^2).
        """
        f = self._read_register(18)
        return {
            "accel_x": float(f[0]),
            "accel_y": float(f[1]),
            "accel_z": float(f[2]),
        }

    def read_accel_x(self) -> float:
        """Read compensated accelerometer X-axis in m/s^2."""
        return self.read_accel()["accel_x"]

    def read_accel_y(self) -> float:
        """Read compensated accelerometer Y-axis in m/s^2."""
        return self.read_accel()["accel_y"]

    def read_accel_z(self) -> float:
        """Read compensated accelerometer Z-axis in m/s^2."""
        return self.read_accel()["accel_z"]

    def read_gyro(self) -> dict:
        """Read compensated angular rate (gyroscope) measurement.

        Register 19.

        Returns:
            dict with keys: gyro_x, gyro_y, gyro_z (all in rad/s).
        """
        f = self._read_register(19)
        return {
            "gyro_x": float(f[0]),
            "gyro_y": float(f[1]),
            "gyro_z": float(f[2]),
        }

    def read_gyro_x(self) -> float:
        """Read compensated angular rate X-axis in rad/s."""
        return self.read_gyro()["gyro_x"]

    def read_gyro_y(self) -> float:
        """Read compensated angular rate Y-axis in rad/s."""
        return self.read_gyro()["gyro_y"]

    def read_gyro_z(self) -> float:
        """Read compensated angular rate Z-axis in rad/s."""
        return self.read_gyro()["gyro_z"]

    # ── Combined compensated sensors (register 20) ───────────────────

    def read_mag_accel_gyro(self) -> dict:
        """Read all compensated magnetic, acceleration, and angular rate data.

        Register 20. Single register read for efficiency.

        Returns:
            dict with keys:
                mag_x, mag_y, mag_z         (Gauss)
                accel_x, accel_y, accel_z   (m/s^2)
                gyro_x, gyro_y, gyro_z      (rad/s)
        """
        f = self._read_register(20)
        return {
            "mag_x": float(f[0]),
            "mag_y": float(f[1]),
            "mag_z": float(f[2]),
            "accel_x": float(f[3]),
            "accel_y": float(f[4]),
            "accel_z": float(f[5]),
            "gyro_x": float(f[6]),
            "gyro_y": float(f[7]),
            "gyro_z": float(f[8]),
        }

    # ── Attitude + compensated sensors (register 27) ─────────────────

    def read_ypr_mag_accel_gyro(self) -> dict:
        """Read attitude and all compensated sensor data in one register read.

        Register 27. Most efficient way to get attitude + raw sensor data.

        Returns:
            dict with keys:
                yaw, pitch, roll            (deg)
                mag_x, mag_y, mag_z         (Gauss)
                accel_x, accel_y, accel_z   (m/s^2)
                gyro_x, gyro_y, gyro_z      (rad/s)
        """
        f = self._read_register(27)
        return {
            "yaw": float(f[0]),
            "pitch": float(f[1]),
            "roll": float(f[2]),
            "mag_x": float(f[3]),
            "mag_y": float(f[4]),
            "mag_z": float(f[5]),
            "accel_x": float(f[6]),
            "accel_y": float(f[7]),
            "accel_z": float(f[8]),
            "gyro_x": float(f[9]),
            "gyro_y": float(f[10]),
            "gyro_z": float(f[11]),
        }

    # ── Uncompensated IMU measurements (register 54) ─────────────────

    def read_imu(self) -> dict:
        """Read all uncompensated IMU measurements including temp and pressure.

        Register 54. These are calibrated but not bias-compensated values.

        Returns:
            dict with keys:
                mag_x, mag_y, mag_z         (Gauss)  - uncompensated
                accel_x, accel_y, accel_z   (m/s^2)  - uncompensated
                gyro_x, gyro_y, gyro_z      (rad/s)  - uncompensated
                temperature                 (deg C)
                pressure                    (kPa)
        """
        f = self._read_register(54)
        return {
            "mag_x": float(f[0]),
            "mag_y": float(f[1]),
            "mag_z": float(f[2]),
            "accel_x": float(f[3]),
            "accel_y": float(f[4]),
            "accel_z": float(f[5]),
            "gyro_x": float(f[6]),
            "gyro_y": float(f[7]),
            "gyro_z": float(f[8]),
            "temperature": float(f[9]),
            "pressure": float(f[10]),
        }

    def read_temperature(self) -> float:
        """Read IMU temperature in degrees Celsius."""
        return self.read_imu()["temperature"]

    def read_pressure(self) -> float:
        """Read barometric pressure in kPa."""
        return self.read_imu()["pressure"]

    # ── Delta theta and delta velocity (register 80) ─────────────────

    def read_delta_theta_velocity(self) -> dict:
        """Read coning and sculling integrated delta values.

        Register 80. Values accumulate since last read (polling mode).

        Returns:
            dict with keys:
                delta_time                              (sec)
                delta_theta_x, delta_theta_y, delta_theta_z (deg)
                delta_velocity_x, delta_velocity_y, delta_velocity_z (m/s)
        """
        f = self._read_register(80)
        return {
            "delta_time": float(f[0]),
            "delta_theta_x": float(f[1]),
            "delta_theta_y": float(f[2]),
            "delta_theta_z": float(f[3]),
            "delta_velocity_x": float(f[4]),
            "delta_velocity_y": float(f[5]),
            "delta_velocity_z": float(f[6]),
        }

    # ── True body acceleration (register 239) ────────────────────────

    def read_ypr_body_accel_gyro(self) -> dict:
        """Read attitude, true body acceleration (no gravity), and angular rates.

        Register 239. Body acceleration has gravity removed - reads ~0 when
        stationary, unlike register 18 which includes gravity.

        Returns:
            dict with keys:
                yaw, pitch, roll                        (deg)
                body_accel_x, body_accel_y, body_accel_z (m/s^2, no gravity)
                gyro_x, gyro_y, gyro_z                  (rad/s)
        """
        f = self._read_register(239)
        return {
            "yaw": float(f[0]),
            "pitch": float(f[1]),
            "roll": float(f[2]),
            "body_accel_x": float(f[3]),
            "body_accel_y": float(f[4]),
            "body_accel_z": float(f[5]),
            "gyro_x": float(f[6]),
            "gyro_y": float(f[7]),
            "gyro_z": float(f[8]),
        }

    def read_body_accel_x(self) -> float:
        """Read linear body acceleration X-axis in m/s^2 (gravity removed)."""
        return self.read_ypr_body_accel_gyro()["body_accel_x"]

    def read_body_accel_y(self) -> float:
        """Read linear body acceleration Y-axis in m/s^2 (gravity removed)."""
        return self.read_ypr_body_accel_gyro()["body_accel_y"]

    def read_body_accel_z(self) -> float:
        """Read linear body acceleration Z-axis in m/s^2 (gravity removed)."""
        return self.read_ypr_body_accel_gyro()["body_accel_z"]

    # ── GPS Solution LLA (register 58) ───────────────────────────────

    def read_gps(self) -> dict:
        """Read the GPS solution in latitude/longitude/altitude.

        Register 58. Updated at 5 Hz by the onboard GPS receiver.

        Returns:
            dict with keys:
                gps_time        (sec)   - GPS time of week
                gps_week        (week)  - GPS week number
                gps_fix         (int)   - 0=No fix, 1=Time only, 2=2D, 3=3D
                num_satellites  (int)   - Number of satellites used
                latitude        (deg)   - Geodetic latitude (WGS84)
                longitude       (deg)   - Geodetic longitude (WGS84)
                altitude        (m)     - Height above WGS84 ellipsoid
                ned_vel_x       (m/s)   - Velocity north
                ned_vel_y       (m/s)   - Velocity east
                ned_vel_z       (m/s)   - Velocity down
                north_acc       (m)     - North position accuracy estimate
                east_acc        (m)     - East position accuracy estimate
                vert_acc        (m)     - Vertical position accuracy estimate
                speed_acc       (m/s)   - Speed accuracy estimate
                time_acc        (sec)   - Time accuracy estimate
        """
        f = self._read_register(58)
        return {
            "gps_time": float(f[0]),
            "gps_week": int(f[1]),
            "gps_fix": int(f[2]),
            "num_satellites": int(f[3]),
            "latitude": float(f[4]),
            "longitude": float(f[5]),
            "altitude": float(f[6]),
            "ned_vel_x": float(f[7]),
            "ned_vel_y": float(f[8]),
            "ned_vel_z": float(f[9]),
            "north_acc": float(f[10]),
            "east_acc": float(f[11]),
            "vert_acc": float(f[12]),
            "speed_acc": float(f[13]),
            "time_acc": float(f[14]),
        }

    def read_gps_fix(self) -> int:
        """Read GPS fix type. 0=No fix, 1=Time only, 2=2D, 3=3D."""
        return self.read_gps()["gps_fix"]

    def read_gps_num_satellites(self) -> int:
        """Read number of GPS satellites used in solution."""
        return self.read_gps()["num_satellites"]

    def read_gps_latitude(self) -> float:
        """Read GPS latitude in degrees (WGS84)."""
        return self.read_gps()["latitude"]

    def read_gps_longitude(self) -> float:
        """Read GPS longitude in degrees (WGS84)."""
        return self.read_gps()["longitude"]

    def read_gps_altitude(self) -> float:
        """Read GPS altitude in meters above WGS84 ellipsoid."""
        return self.read_gps()["altitude"]

    def read_gps_ned_velocity(self) -> dict:
        """Read GPS velocity in NED frame.

        Returns:
            dict with keys: ned_vel_x (north), ned_vel_y (east),
            ned_vel_z (down) in m/s.
        """
        gps = self.read_gps()
        return {
            "ned_vel_x": gps["ned_vel_x"],
            "ned_vel_y": gps["ned_vel_y"],
            "ned_vel_z": gps["ned_vel_z"],
        }

    # ── GPS Solution ECEF (register 59) ──────────────────────────────

    def read_gps_ecef(self) -> dict:
        """Read GPS solution in Earth-Centered Earth-Fixed (ECEF) frame.

        Register 59. Updated at 5 Hz.

        Returns:
            dict with keys:
                gps_time        (sec)   - GPS time of week
                gps_week        (week)  - GPS week number
                gps_fix         (int)   - 0=No fix, 1=Time only, 2=2D, 3=3D
                num_satellites  (int)   - Number of satellites used
                position_x      (m)     - ECEF X coordinate
                position_y      (m)     - ECEF Y coordinate
                position_z      (m)     - ECEF Z coordinate
                velocity_x      (m/s)   - ECEF X velocity
                velocity_y      (m/s)   - ECEF Y velocity
                velocity_z      (m/s)   - ECEF Z velocity
                pos_acc_x       (m)     - ECEF X position accuracy estimate
                pos_acc_y       (m)     - ECEF Y position accuracy estimate
                pos_acc_z       (m)     - ECEF Z position accuracy estimate
                speed_acc       (m/s)   - Speed accuracy estimate
                time_acc        (sec)   - Time accuracy estimate
        """
        f = self._read_register(59)
        return {
            "gps_time": float(f[0]),
            "gps_week": int(f[1]),
            "gps_fix": int(f[2]),
            "num_satellites": int(f[3]),
            "position_x": float(f[4]),
            "position_y": float(f[5]),
            "position_z": float(f[6]),
            "velocity_x": float(f[7]),
            "velocity_y": float(f[8]),
            "velocity_z": float(f[9]),
            "pos_acc_x": float(f[10]),
            "pos_acc_y": float(f[11]),
            "pos_acc_z": float(f[12]),
            "speed_acc": float(f[13]),
            "time_acc": float(f[14]),
        }

    # ── INS Solution LLA (register 63) ───────────────────────────────

    def read_ins(self) -> dict:
        """Read INS Kalman filter solution in LLA coordinates.

        Register 63. Fuses GPS + IMU for the best combined estimate.
        Updated at the full NavFilter rate (up to 400 Hz).

        Returns:
            dict with keys:
                ins_time            (sec)   - GPS time of week
                ins_week            (week)  - GPS week number
                ins_status          (int)   - Status bitfield (hex). See parse_ins_status().
                yaw, pitch, roll    (deg)   - Attitude relative to true north / horizon
                latitude            (deg)   - INS geodetic latitude (WGS84)
                longitude           (deg)   - INS geodetic longitude (WGS84)
                altitude            (m)     - INS height above WGS84 ellipsoid
                ned_vel_x           (m/s)   - INS velocity north
                ned_vel_y           (m/s)   - INS velocity east
                ned_vel_z           (m/s)   - INS velocity down
                att_uncertainty     (deg)   - Attitude uncertainty estimate
                pos_uncertainty     (m)     - Position uncertainty estimate
                vel_uncertainty     (m/s)   - Velocity uncertainty estimate
        """
        f = self._read_register(63)
        return {
            "ins_time": float(f[0]),
            "ins_week": int(f[1]),
            "ins_status": int(f[2], 16),
            "yaw": float(f[3]),
            "pitch": float(f[4]),
            "roll": float(f[5]),
            "latitude": float(f[6]),
            "longitude": float(f[7]),
            "altitude": float(f[8]),
            "ned_vel_x": float(f[9]),
            "ned_vel_y": float(f[10]),
            "ned_vel_z": float(f[11]),
            "att_uncertainty": float(f[12]),
            "pos_uncertainty": float(f[13]),
            "vel_uncertainty": float(f[14]),
        }

    def read_ins_latitude(self) -> float:
        """Read INS-fused latitude in degrees (WGS84)."""
        return self.read_ins()["latitude"]

    def read_ins_longitude(self) -> float:
        """Read INS-fused longitude in degrees (WGS84)."""
        return self.read_ins()["longitude"]

    def read_ins_altitude(self) -> float:
        """Read INS-fused altitude in meters above WGS84 ellipsoid."""
        return self.read_ins()["altitude"]

    def read_ins_ned_velocity(self) -> dict:
        """Read INS-fused velocity in NED frame.

        Returns:
            dict with keys: ned_vel_x (north), ned_vel_y (east),
            ned_vel_z (down) in m/s.
        """
        ins = self.read_ins()
        return {
            "ned_vel_x": ins["ned_vel_x"],
            "ned_vel_y": ins["ned_vel_y"],
            "ned_vel_z": ins["ned_vel_z"],
        }

    # ── INS Solution ECEF (register 64) ──────────────────────────────

    def read_ins_ecef(self) -> dict:
        """Read INS Kalman filter solution in ECEF coordinates.

        Register 64.

        Returns:
            dict with keys:
                ins_time            (sec)   - GPS time of week
                ins_week            (week)  - GPS week number
                ins_status          (int)   - Status bitfield
                yaw, pitch, roll    (deg)   - Attitude
                position_x/y/z     (m)     - ECEF position
                velocity_x/y/z     (m/s)   - ECEF velocity
                att_uncertainty     (deg)   - Attitude uncertainty
                pos_uncertainty     (m)     - Position uncertainty
                vel_uncertainty     (m/s)   - Velocity uncertainty
        """
        f = self._read_register(64)
        return {
            "ins_time": float(f[0]),
            "ins_week": int(f[1]),
            "ins_status": int(f[2], 16),
            "yaw": float(f[3]),
            "pitch": float(f[4]),
            "roll": float(f[5]),
            "position_x": float(f[6]),
            "position_y": float(f[7]),
            "position_z": float(f[8]),
            "velocity_x": float(f[9]),
            "velocity_y": float(f[10]),
            "velocity_z": float(f[11]),
            "att_uncertainty": float(f[12]),
            "pos_uncertainty": float(f[13]),
            "vel_uncertainty": float(f[14]),
        }

    # ── INS State LLA (register 72) ──────────────────────────────────

    def read_ins_state_lla(self) -> dict:
        """Read full INS state including body-frame accel and angular rates.

        Register 72. All-in-one register for INS state in LLA coordinates.

        Returns:
            dict with keys:
                yaw, pitch, roll                        (deg)
                latitude, longitude                     (deg, WGS84)
                altitude                                (m, WGS84)
                velocity_x (north), velocity_y (east), velocity_z (down) (m/s)
                accel_x, accel_y, accel_z               (m/s^2, body frame)
                angular_rate_x/y/z                      (rad/s, body frame)
        """
        f = self._read_register(72)
        return {
            "yaw": float(f[0]),
            "pitch": float(f[1]),
            "roll": float(f[2]),
            "latitude": float(f[3]),
            "longitude": float(f[4]),
            "altitude": float(f[5]),
            "velocity_x": float(f[6]),
            "velocity_y": float(f[7]),
            "velocity_z": float(f[8]),
            "accel_x": float(f[9]),
            "accel_y": float(f[10]),
            "accel_z": float(f[11]),
            "angular_rate_x": float(f[12]),
            "angular_rate_y": float(f[13]),
            "angular_rate_z": float(f[14]),
        }

    # ── INS State ECEF (register 73) ─────────────────────────────────

    def read_ins_state_ecef(self) -> dict:
        """Read full INS state in ECEF coordinates.

        Register 73.

        Returns:
            dict with keys:
                yaw, pitch, roll                        (deg)
                position_x/y/z                          (m, ECEF)
                velocity_x/y/z                          (m/s, ECEF)
                accel_x, accel_y, accel_z               (m/s^2, body frame)
                angular_rate_x/y/z                      (rad/s, body frame)
        """
        f = self._read_register(73)
        return {
            "yaw": float(f[0]),
            "pitch": float(f[1]),
            "roll": float(f[2]),
            "position_x": float(f[3]),
            "position_y": float(f[4]),
            "position_z": float(f[5]),
            "velocity_x": float(f[6]),
            "velocity_y": float(f[7]),
            "velocity_z": float(f[8]),
            "accel_x": float(f[9]),
            "accel_y": float(f[10]),
            "accel_z": float(f[11]),
            "angular_rate_x": float(f[12]),
            "angular_rate_y": float(f[13]),
            "angular_rate_z": float(f[14]),
        }

    # ── Quaternion + compensated sensors (register 15) ────────────────

    def read_quat_mag_accel_gyro(self) -> dict:
        """Read quaternion attitude with compensated mag, accel, and gyro.

        Register 15.

        Returns:
            dict with keys:
                q0, q1, q2, q3              (unitless, quaternion)
                mag_x, mag_y, mag_z         (Gauss)
                accel_x, accel_y, accel_z   (m/s^2)
                gyro_x, gyro_y, gyro_z      (rad/s)
        """
        f = self._read_register(15)
        return {
            "q0": float(f[0]),
            "q1": float(f[1]),
            "q2": float(f[2]),
            "q3": float(f[3]),
            "mag_x": float(f[4]),
            "mag_y": float(f[5]),
            "mag_z": float(f[6]),
            "accel_x": float(f[7]),
            "accel_y": float(f[8]),
            "accel_z": float(f[9]),
            "gyro_x": float(f[10]),
            "gyro_y": float(f[11]),
            "gyro_z": float(f[12]),
        }

    # ── Helpers ───────────────────────────────────────────────────────

    @staticmethod
    def parse_ins_status(status: int) -> dict:
        """Parse the INS status bitfield into human-readable fields.

        Args:
            status: The integer status value from read_ins() or read_ins_ecef().

        Returns:
            dict with keys:
                mode    (int)   - 0=Not tracking, 1=Degraded, 2=Tracking
                gps_fix (bool)  - True if GPS has a proper fix
                error   (int)   - Error bitfield (0=no errors)
                    Bit 0: Time error
                    Bit 1: IMU error
                    Bit 2: Mag/Pressure error
                    Bit 3: GPS error
        """
        return {
            "mode": status & 0x03,
            "gps_fix": bool((status >> 2) & 0x01),
            "error": (status >> 3) & 0x0F,
        }

    @staticmethod
    def list_serial_ports() -> list[str]:
        """List available serial ports on the system.

        Returns:
            List of port device names (e.g. ["/dev/ttyUSB0", "COM3"]).
        """
        from serial.tools.list_ports import comports

        return [p.device for p in comports()]
