from __future__ import annotations

import time
from queue import Empty, Queue
from threading import Event, Lock, Thread
from typing import Callable, Optional

from serial import Serial
from serial.serialutil import SerialException, SerialTimeoutException

import Constants
from protocol import *


class ControlService:
    def __init__(self, port: str = Constants.serial_port, baudrate: int = Constants.baud_rate,
                timeout: float = Constants.timeout, debug: bool = False) -> None:
        self.port_name, self.baudrate, self.timeout, self.debug = port, baudrate, timeout, debug
        self.port: Optional[Serial] = None
        self.packet_reader = PacketReader()
        self._response_queue = Queue()
        self._event_queue = Queue()
        self._listener_stop = Event()
        self._listener_thread: Optional[Thread] = None
        self._write_lock = Lock()
        self._command_lock = Lock()

    def connect(self) -> bool:
        while True:
            try:
                self.port = Serial(self.port_name, self.baudrate, timeout=self.timeout, write_timeout=self.timeout)
                self.port.reset_input_buffer()
                self._listener_stop.clear()
                self._listener_thread = Thread(target=self._listen_loop, name="uart-listener", daemon=True)
                self._listener_thread.start()
                return True
            except SerialException as error:
                print(f"Serial exception: {error}")
                time.sleep(1.0)

    def close(self) -> None:
        self._listener_stop.set()
        if self.port is not None and self.port.is_open: self.port.close()
        if self._listener_thread is not None and self._listener_thread.is_alive():
            self._listener_thread.join(timeout=1.0)
        self._listener_thread = None
        self.port, self.packet_reader = None, PacketReader()
        self._response_queue = Queue()
        self._event_queue = Queue()

    disconnect = close
    @property
    def is_connected(self) -> bool: return self.port is not None and self.port.is_open
    def reconnect(self) -> bool: self.close(); return self.connect()

    def move_forward(self, distance_mm: int) -> bool: return self._move(CMD_FORWARD, distance_mm)
    def move_reverse(self, distance_mm: int) -> bool: return self._move(CMD_REVERSE, distance_mm)
    def turn_left(self, angle_degrees: int) -> bool: return self._turn(CMD_TURN_LEFT, angle_degrees)
    def turn_right(self, angle_degrees: int) -> bool: return self._turn(CMD_TURN_RIGHT, angle_degrees)
    def pause(self) -> bool: return self._command(CMD_PAUSE)
    def resume(self) -> bool: return self._command(CMD_RESUME)
    def cancel(self) -> bool: return self._command(CMD_CANCEL)
    def return_to_center(self) -> bool: return self._command(CMD_RETURN_TO_CENTER)
    def drop_kit(self) -> bool: return self._command(CMD_DROP_KIT)
    def ping(self) -> bool: return self._command(CMD_PING)
    def heartbeat(self) -> bool: return self._command(CMD_HEARTBEAT)

    def request_status(self) -> Optional[RobotStatus]:
        with self._command_lock:
            if not self._send(build_command_packet(CMD_GET_STATUS)): return None
            packet = self._wait_for(lambda p: p.command == RSP_STATUS)
        try: return parse_status_packet(packet) if packet else None
        except ValueError as error: print(f"Invalid status: {error}"); return None

    def receive_event(self, timeout: float = 0.0) -> Optional[RobotEvent]:
        try: return self._event_queue.get(timeout=timeout)
        except Empty: return None

    def _listen_loop(self) -> None:
        while not self._listener_stop.is_set():
            packet = self._read_packet()
            if packet is None:
                continue
            if packet.command == RSP_EVENT:
                try: self._event_queue.put(parse_event_packet(packet))
                except ValueError as error: print(f"Invalid event: {error}")
            else:
                self._response_queue.put(packet)

    def _read_packet(self) -> Optional[Packet]:
        if not self.is_connected: return None
        try: data = self.port.read(1)
        except SerialException as error:
            if not self._listener_stop.is_set(): print(f"Serial reception error: {error}")
            return None
        packet = self.packet_reader.feed(data[0]) if data else None
        if packet and self.debug: print(f"[RX] {rebuild_packet(packet).hex(' ').upper()}")
        return packet

    def _command(self, command: int) -> bool: return self._send_and_confirm(build_command_packet(command), command)
    def _move(self, command: int, distance_mm: int) -> bool: return self._send_and_confirm(build_move_packet(command, distance_mm), command)
    def _turn(self, command: int, angle_degrees: int) -> bool: return self._send_and_confirm(build_turn_packet(command, angle_degrees), command)
    def _send(self, packet: bytes) -> bool:
        if not self.is_connected: return False
        try:
            with self._write_lock:
                written = self.port.write(packet) 
            if self.debug: print(f"[TX] {packet.hex(' ').upper()}")
            return written == len(packet)
        except (SerialException, SerialTimeoutException) as error: print(f"Serial transmission error: {error}"); self.close(); return False

    def _send_and_confirm(self, packet: bytes, command: int) -> bool:
        with self._command_lock:
            if not self._send(packet): return False
            response = self._wait_for(lambda p: p.command in (RSP_ACK, RSP_NACK) and p.payload_len in (2, 3) and p.payload[1] == command)
        if response is None: return False
        if response.command == RSP_ACK: return parse_ack_packet(response) == command
        rejected, error = parse_nack_packet(response); print(f"Command 0x{rejected:02X} rejected, error {error}"); return False

    def _wait_for(self, predicate: Callable[[Packet], bool]) -> Optional[Packet]:
        deadline = time.monotonic() + Constants.response_timeout
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            try: packet = self._response_queue.get(timeout=remaining)
            except Empty: return None
            if packet and predicate(packet): return packet
            self._response_queue.put(packet)
            time.sleep(0.001)
        return None
