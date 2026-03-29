#!/usr/bin/env python3
"""
MKS SERVO42C UART Protocol Helper.

Implements the binary UART protocol for MKS SERVO42C closed-loop stepper motor.

Hardware:
  - MKS SERVO42C — 14-bit magnetic encoder + closed-loop stepper driver
  - MKS APT adapter board + 6-pin data cable
  - Connect to PC/RPi via USB-TTL converter

UART Protocol frame format:
  [Slave_Addr] [Command] [Data...] [CRC]
  CRC = (Slave_Addr + Command + sum(data_bytes)) & 0xFF

Default settings:
  Baud rate : 38400 (configurable 9600/19200/38400/57600/115200/256000)
  Slave addr: 0x00  (configurable 0x00–0xFF per motor)
  Mode      : Must be set to CR_UART (mode 3) to accept UART position commands

Encoder resolution: 16384 counts per revolution (14-bit)
Motor resolution  : 200 steps/rev × subdivision = steps/rev
                    (default 16 microsteps → 3200 steps/rev)

Reference: MKS SERVO42C User Manual (UART section)
"""

import struct
import time


# ─────────────────────────── Command bytes ───────────────────────────────────

class Cmd:
    # Configuration
    SET_BAUD       = 0x80   # Set baud rate
    SET_ADDR       = 0x81   # Set slave address
    SET_MODE       = 0x82   # Set working mode (0=open-loop, 3=uart-closed-loop)
    SET_CURRENT    = 0x83   # Set working current (mA)
    SET_SUBDIV     = 0x84   # Set microstep subdivision
    SET_DIR        = 0x85   # Set motor direction (0/1)
    SET_EN_ACTIVE  = 0x86   # Set enable-pin active level (0=low, 1=high)
    SET_AUTO_LOCK  = 0x87   # Auto-lock shaft after idle
    SET_SHAFT_LOCK = 0x88   # Lock shaft immediately

    # Status query
    READ_ENCODER   = 0x30   # Read real-time encoder value (16384 cpr)
    READ_SPEED     = 0x31   # Read real-time motor speed (RPM)
    READ_PULSES    = 0x32   # Read total number of pulses received
    READ_IO        = 0x33   # Read IO port state
    READ_FLAGS     = 0x3A   # Read error flags (stall, etc.)
    QUERY_STATUS   = 0xA1   # Query motor running status

    # Control
    ENABLE         = 0xF3   # Enable (0x01) / Disable (0x00) motor
    SET_POS_ZERO   = 0x93   # Set current position as zero (data byte = 0x88)
    MOVE_SPEED     = 0xF6   # Speed/direction move (continuous)
    MOVE_ABSOLUTE  = 0xFE   # Move to absolute position (pulses from zero)
    MOVE_RELATIVE  = 0xFD   # Move by relative pulses
    STOP           = 0xF7   # Emergency stop
    SAVE_PARAMS    = 0xFF   # Save parameters to flash


# ─────────────────────────── Mode codes ──────────────────────────────────────

class Mode:
    CR_OPEN       = 0   # Open-loop pulse control (step/dir pins)
    CR_CLOSE      = 1   # Closed-loop pulse control (step/dir pins)
    CR_vFOC       = 2   # Velocity FOC (step/dir pins, smooth torque)
    CR_UART       = 3   # UART control mode  ← use this for direct serial


# ─────────────────────────── Status codes ────────────────────────────────────

class MotorStatus:
    STOPPED   = 0x00
    ACCEL     = 0x01
    DECEL     = 0x02
    RUNNING   = 0x03
    HOMING    = 0x04
    CALIB     = 0x05
    STALL     = 0x06  # Fault: shaft stalled
    UNKNOWN   = 0xFF


# ─────────────────────────── Baud-rate codes ─────────────────────────────────

BAUD_CODES = {
    9600:   0x00,
    19200:  0x01,
    38400:  0x02,   # default
    57600:  0x03,
    115200: 0x04,
    256000: 0x05,
}

ENCODER_COUNTS_PER_REV = 16384  # 14-bit encoder


# ─────────────────────────── Protocol helper ─────────────────────────────────

class MksServo42cProtocol:
    """
    Low-level UART protocol for MKS SERVO42C.

    Usage:
        proto = MksServo42cProtocol(addr=0x00)
        frame = proto.build_read_encoder()
        ser.write(frame)
        resp  = ser.read(8)
        angle = proto.parse_encoder_response(resp)
    """

    def __init__(self, addr: int = 0x00):
        self.addr = addr & 0xFF

    # ── CRC ──────────────────────────────────────────────────────────────────

    def _crc(self, *bytes_iter) -> int:
        """Compute 1-byte checksum: sum of all bytes, take lower 8 bits."""
        return sum(bytes_iter) & 0xFF

    # ── Frame builders ────────────────────────────────────────────────────────

    def _frame(self, cmd: int, data: bytes = b'') -> bytes:
        """Build a complete protocol frame with CRC."""
        crc = self._crc(self.addr, cmd, *data)
        return bytes([self.addr, cmd]) + data + bytes([crc])

    def build_read_encoder(self) -> bytes:
        """Request frame: read real-time encoder value (14-bit wraparound)."""
        return self._frame(Cmd.READ_ENCODER)

    def build_read_speed(self) -> bytes:
        """Request frame: read motor speed in RPM."""
        return self._frame(Cmd.READ_SPEED)

    def build_query_status(self) -> bytes:
        """Request frame: query motor running status."""
        return self._frame(Cmd.QUERY_STATUS)

    def build_read_flags(self) -> bytes:
        """Request frame: read error/stall flags."""
        return self._frame(Cmd.READ_FLAGS)

    def build_enable(self, enable: bool = True) -> bytes:
        """Request frame: enable or disable motor torque."""
        return self._frame(Cmd.ENABLE, bytes([0x01 if enable else 0x00]))

    def build_set_zero(self) -> bytes:
        """Request frame: set current position as encoder zero."""
        return self._frame(Cmd.SET_POS_ZERO, bytes([0x88]))

    def build_stop(self) -> bytes:
        """Request frame: emergency stop."""
        return self._frame(Cmd.STOP)

    def build_set_mode(self, mode: int = Mode.CR_UART) -> bytes:
        """Request frame: set working mode (must be CR_UART=3 for UART commands)."""
        return self._frame(Cmd.SET_MODE, bytes([mode]))

    def build_set_subdivision(self, subdivision: int = 16) -> bytes:
        """
        Request frame: set microstep subdivision.
        Valid values: 1, 2, 4, 8, 16, 32, 64, 128, 256
        """
        return self._frame(Cmd.SET_SUBDIV, bytes([subdivision]))

    def build_set_current(self, current_ma: int = 1000) -> bytes:
        """
        Request frame: set motor working current in mA.
        Range: 0–3000 mA (NEMA17 typically 1000–1500 mA)
        """
        high = (current_ma >> 8) & 0xFF
        low  = current_ma & 0xFF
        return self._frame(Cmd.SET_CURRENT, bytes([high, low]))

    def build_save_params(self) -> bytes:
        """Request frame: save current parameters to flash."""
        return self._frame(Cmd.SAVE_PARAMS, bytes([0xFF]))

    def build_move_relative(self, speed_pct: float, pulses: int,
                             clockwise: bool = True) -> bytes:
        """
        Request frame: move by relative number of pulses.

        Args:
            speed_pct : Desired speed as fraction 0.0–1.0 of MAX_SPEED
            pulses    : Number of microstep pulses (positive integer)
            clockwise : Direction (True=CW, False=CCW)

        Frame: [Addr][0xFD][dir_speed][pulses2][pulses1][pulses0][CRC]
          dir_speed  bits: [7]=direction(0=CW,1=CCW), [6:0]=speed (1–127)
          pulses     : 3-byte unsigned big-endian
        """
        speed_byte = max(1, min(127, int(speed_pct * 127)))
        if not clockwise:
            speed_byte |= 0x80   # set bit 7 for CCW

        pulses = max(0, int(pulses))
        p2 = (pulses >> 16) & 0xFF
        p1 = (pulses >>  8) & 0xFF
        p0 =  pulses        & 0xFF

        return self._frame(Cmd.MOVE_RELATIVE, bytes([speed_byte, p2, p1, p0]))

    def build_move_absolute(self, speed_pct: float, pulses: int,
                             clockwise: bool = True) -> bytes:
        """
        Request frame: move to absolute pulse position from zero.

        Same encoding as move_relative but uses 0xFE command.
        """
        speed_byte = max(1, min(127, int(speed_pct * 127)))
        if not clockwise:
            speed_byte |= 0x80

        pulses = max(0, int(pulses))
        p2 = (pulses >> 16) & 0xFF
        p1 = (pulses >>  8) & 0xFF
        p0 =  pulses        & 0xFF

        return self._frame(Cmd.MOVE_ABSOLUTE, bytes([speed_byte, p2, p1, p0]))

    def build_move_speed(self, speed_pct: float, clockwise: bool = True) -> bytes:
        """
        Request frame: continuous speed mode.

        Args:
            speed_pct: 0.0–1.0 fraction of max speed
            clockwise: direction
        """
        speed_byte = max(0, min(127, int(speed_pct * 127)))
        if not clockwise:
            speed_byte |= 0x80
        return self._frame(Cmd.MOVE_SPEED, bytes([speed_byte]))

    # ── Response parsers ──────────────────────────────────────────────────────

    def _validate_crc(self, frame: bytes) -> bool:
        """Verify CRC of a received frame."""
        if len(frame) < 2:
            return False
        expected = self._crc(*frame[:-1])
        return frame[-1] == expected

    def parse_encoder_response(self, frame: bytes):
        """
        Parse encoder read response.

        Expected: [Addr][0x30][Carry][Val3][Val2][Val1][Val0][CRC]  (8 bytes)

        Returns:
            (encoder_raw, angle_deg) or None on error.

        Carry: signed int8, counts full 16384-unit revolutions.
        Val0-Val3: unsigned 32-bit encoder value within one revolution (0–16383).

        Total encoder count = Carry * 16384 + unsigned_val
        Angle = total_count / 16384 * 360  (degrees, multi-turn)
        """
        if frame is None or len(frame) < 8:
            return None
        if frame[0] != self.addr or frame[1] != Cmd.READ_ENCODER:
            return None
        if not self._validate_crc(frame):
            return None

        carry = struct.unpack('b', bytes([frame[2]]))[0]   # signed byte
        val   = (frame[3] << 24) | (frame[4] << 16) | (frame[5] << 8) | frame[6]

        total_counts = carry * ENCODER_COUNTS_PER_REV + val
        angle_deg    = total_counts / ENCODER_COUNTS_PER_REV * 360.0
        return total_counts, angle_deg

    def parse_speed_response(self, frame: bytes):
        """
        Parse speed read response.

        Expected: [Addr][0x31][direction][speed_H][speed_L][CRC]  (6 bytes)

        Returns:
            (speed_rpm, clockwise) or None on error.
        """
        if frame is None or len(frame) < 6:
            return None
        if frame[0] != self.addr or frame[1] != Cmd.READ_SPEED:
            return None
        if not self._validate_crc(frame):
            return None

        direction = frame[2]  # 0=CW, 1=CCW
        speed_rpm = (frame[3] << 8) | frame[4]
        return speed_rpm, (direction == 0)

    def parse_status_response(self, frame: bytes):
        """
        Parse status query response.

        Expected: [Addr][0xA1][status][CRC]  (4 bytes)

        Returns:
            status byte (see MotorStatus) or None on error.
        """
        if frame is None or len(frame) < 4:
            return None
        if frame[0] != self.addr or frame[1] != Cmd.QUERY_STATUS:
            return None
        if not self._validate_crc(frame):
            return None
        return frame[2]

    def parse_ack(self, frame: bytes, cmd: int) -> bool:
        """
        Parse a simple ACK response.

        Expected: [Addr][Cmd][0x01=success/0x00=fail][CRC]  (4 bytes)

        Returns:
            True if success, False otherwise.
        """
        if frame is None or len(frame) < 4:
            return False
        if frame[0] != self.addr or frame[1] != cmd:
            return False
        if not self._validate_crc(frame):
            return False
        return frame[2] == 0x01

    def degrees_to_pulses(self, angle_deg: float, subdivision: int = 16) -> int:
        """
        Convert an angle in degrees to motor pulses (microsteps).

        Args:
            angle_deg:   Target angle in degrees
            subdivision: Microstep subdivision setting on motor

        Returns:
            Integer number of pulses (steps) for the given angle.

        Formula:
            pulses = (angle / 360) * 200_steps_per_rev * subdivision
        """
        steps_per_rev = 200 * subdivision
        return int(round(angle_deg / 360.0 * steps_per_rev))

    def pulses_to_degrees(self, pulses: int, subdivision: int = 16) -> float:
        """
        Convert motor pulses (microsteps) to degrees.

        Args:
            pulses:      Number of microstep pulses
            subdivision: Microstep subdivision setting on motor

        Returns:
            Angle in degrees.
        """
        steps_per_rev = 200 * subdivision
        return pulses / steps_per_rev * 360.0

    def encoder_to_degrees(self, encoder_counts: int) -> float:
        """
        Convert raw encoder counts (14-bit, multi-turn) to degrees.

        Args:
            encoder_counts: Raw encoder count (signed, multi-turn)

        Returns:
            Angle in degrees (multi-turn aware).
        """
        return encoder_counts / ENCODER_COUNTS_PER_REV * 360.0
