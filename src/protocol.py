from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional, Tuple
import Constants

CMD_FORWARD = 0x30
CMD_REVERSE = 0x31
CMD_TURN_LEFT = 0x32
CMD_TURN_RIGHT = 0x33
CMD_PAUSE = 0x34
CMD_RESUME = 0x35
CMD_CANCEL = 0x36
CMD_RETURN_TO_CENTER = 0x37
CMD_DROP_KIT = 0x38
CMD_GET_STATUS = 0x39
CMD_PING = 0x3A
CMD_HEARTBEAT = 0x3B

RSP_ACK, RSP_NACK, RSP_STATUS, RSP_EVENT = 0x80, 0x81, 0x82, 0x83

STATE_IDLE, STATE_RUNNING, STATE_PAUSED, STATE_DROPPING_KIT, STATE_FAULT = range(5)
EVENT_COMPLETED, EVENT_BLOCKED_BY_WALL, EVENT_VICTIM_DETECTED, EVENT_KIT_DROPPED, EVENT_FAULT = range(1, 6)

ERR_BAD_LENGTH = 1
ERR_UNKNOWN_COMMAND = 3
ERR_HARDWARE_NOT_READY = 4
ERR_NOTHING_TO_RESUME = 5
ERR_BUSY = 6


@dataclass(frozen=True)
class RobotStatus:
    state: int
    active_command: int
    progress_per_mille: int


@dataclass(frozen=True)
class RobotEvent:
    event: int
    command: int
    progress_per_mille: int


@dataclass(frozen=True)
class Packet:
    payload_len: int
    payload: bytes

    @property
    def command(self) -> int:
        return self.payload[0]


class PacketReader:
    WAIT_HEADER0, WAIT_HEADER1, WAIT_LENGTH, WAIT_PAYLOAD, WAIT_CHECKSUM = range(5)

    def __init__(self) -> None:
        self.state = self.WAIT_HEADER0
        self.payload_len = 0
        self.payload = bytearray()
        self.checksum = 0

    def feed(self, byte: int) -> Optional[Packet]:
        if self.state == self.WAIT_HEADER0:
            if byte == Constants.header0: self.state = self.WAIT_HEADER1
            return None
        if self.state == self.WAIT_HEADER1:
            self.state = self.WAIT_LENGTH if byte == Constants.header1 else (self.WAIT_HEADER1 if byte == Constants.header0 else self.WAIT_HEADER0)
            return None
        if self.state == self.WAIT_LENGTH:
            if not 0 < byte <= Constants.max_payload:
                self.state = self.WAIT_HEADER0
                return None
            self.payload_len, self.payload, self.checksum, self.state = byte, bytearray(), byte, self.WAIT_PAYLOAD
            return None
        if self.state == self.WAIT_PAYLOAD:
            self.payload.append(byte); self.checksum = (self.checksum + byte) & 0xFF
            if len(self.payload) == self.payload_len: self.state = self.WAIT_CHECKSUM
            return None
        self.state = self.WAIT_HEADER0
        return Packet(self.payload_len, bytes(self.payload)) if byte == self.checksum else None


def build_packet(payload: bytes) -> bytes:
    if not 0 < len(payload) <= Constants.max_payload: raise ValueError("Payload must be 1..128 bytes")
    return bytes((Constants.header0, Constants.header1, len(payload))) + payload + bytes(((len(payload) + sum(payload)) & 0xFF,))


def build_command_packet(command: int) -> bytes:
    return build_packet(bytes((command,)))


def _build_magnitude_packet(command: int, magnitude: int, unit: str) -> bytes:
    if not isinstance(magnitude, int) or not 1 <= magnitude <= 65535:
        raise ValueError(f"{unit} must be an integer from 1 to 65535")
    return build_packet(struct.pack("<BH", command, magnitude))


def build_move_packet(command: int, distance_mm: int) -> bytes:
    if command not in (CMD_FORWARD, CMD_REVERSE):
        raise ValueError("Move command must be CMD_FORWARD or CMD_REVERSE")
    return _build_magnitude_packet(command, distance_mm, "distance_mm")


def build_turn_packet(command: int, angle_degrees: int) -> bytes:
    if command not in (CMD_TURN_LEFT, CMD_TURN_RIGHT):
        raise ValueError("Turn command must be CMD_TURN_LEFT or CMD_TURN_RIGHT")
    return _build_magnitude_packet(command, angle_degrees, "angle_degrees")


def parse_ack_packet(packet: Packet) -> int:
    if packet.payload_len != 2 or packet.command != RSP_ACK: raise ValueError("Packet is not an ACK")
    return packet.payload[1]


def parse_nack_packet(packet: Packet) -> Tuple[int, int]:
    if packet.payload_len != 3 or packet.command != RSP_NACK: raise ValueError("Packet is not a NACK")
    return packet.payload[1], packet.payload[2]


def parse_status_packet(packet: Packet) -> RobotStatus:
    if packet.payload_len != 5 or packet.command != RSP_STATUS: raise ValueError("Invalid status")
    return RobotStatus(packet.payload[1], packet.payload[2], struct.unpack_from("<H", packet.payload, 3)[0])


def parse_event_packet(packet: Packet) -> RobotEvent:
    if packet.payload_len != 5 or packet.command != RSP_EVENT: raise ValueError("Invalid event")
    return RobotEvent(packet.payload[1], packet.payload[2], struct.unpack_from("<H", packet.payload, 3)[0])


def rebuild_packet(packet: Packet) -> bytes:
    return build_packet(packet.payload)
