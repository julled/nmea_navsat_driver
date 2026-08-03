"""Tests for the UDP socket driver's handling of quiet sources and bind failures.

The ROS plumbing is stubbed out so the connection/recv loop itself is exercised
without needing a ROS graph.
"""

import socket
import threading
import time

import pytest

from libnmea_navsat_driver.nodes import nmea_socket_driver

GGA = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"


class _FakeParameter:
    def __init__(self, value):
        self.value = value


class _FakeLogger:
    def __init__(self):
        self.errors = []
        self.warnings = []

    def info(self, msg):
        pass

    def warn(self, msg):
        self.warnings.append(msg)

    def error(self, msg):
        self.errors.append(msg)


class _FakeDriver:
    """Stands in for Ros2NMEADriver."""

    def __init__(self, params):
        self._params = params
        self._logger = _FakeLogger()
        self.sentences = []

    def declare_parameter(self, name, default):
        return _FakeParameter(self._params.get(name, default))

    def get_frame_id(self):
        return 'gps'

    def get_logger(self):
        return self._logger

    def add_sentence(self, sentence, frame_id):
        self.sentences.append(sentence)


class _FakeRclpy:
    def __init__(self):
        self.running = True

    def init(self, args=None):
        pass

    def ok(self):
        return self.running


def _free_udp_port():
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(('127.0.0.1', 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


@pytest.fixture
def run_driver(monkeypatch):
    """Start main() in a background thread against stubbed ROS plumbing."""
    rclpy_stub = _FakeRclpy()
    monkeypatch.setattr(nmea_socket_driver, 'rclpy', rclpy_stub)
    # raising=False so the tests still exercise behaviour, rather than erroring
    # out on a missing symbol, if the retry delay is ever renamed or removed
    monkeypatch.setattr(nmea_socket_driver, 'RECONNECT_DELAY_SEC', 0.1, raising=False)

    threads = []

    def start(**params):
        driver = _FakeDriver(params)
        monkeypatch.setattr(nmea_socket_driver, 'Ros2NMEADriver', lambda: driver)
        thread = threading.Thread(target=nmea_socket_driver.main, daemon=True)
        thread.start()
        threads.append(thread)
        return thread, driver

    yield start

    rclpy_stub.running = False
    for thread in threads:
        thread.join(timeout=5)


def test_quiet_source_does_not_tear_down_the_socket(run_driver):
    """A gap longer than timeout_sec is normal, not a recv failure."""
    port = _free_udp_port()
    thread, driver = run_driver(ip='127.0.0.1', port=port, timeout_sec=1, buffer_size=4096)

    # stay silent across several timeouts
    time.sleep(2.5)

    assert driver._logger.errors == [], "recv timeout was treated as a socket error"

    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender.sendto(GGA.encode('ascii') + b'\n', ('127.0.0.1', port))
    sender.close()

    deadline = time.time() + 5
    while not driver.sentences and time.time() < deadline:
        time.sleep(0.05)

    assert driver.sentences == [GGA], "socket stopped receiving after the quiet period"
    assert driver._logger.errors == []
    assert thread.is_alive()


def test_bind_failure_retries_instead_of_exiting(run_driver):
    """A socket that cannot be set up must not take the node down for good."""
    # 192.0.2.0/24 is TEST-NET-1, so it is never a local address
    thread, driver = run_driver(
        ip='192.0.2.1', port=_free_udp_port(), timeout_sec=1, buffer_size=4096)

    time.sleep(0.6)

    assert thread.is_alive(), "node exited on bind failure instead of retrying"
    assert len(driver._logger.errors) >= 2, "expected repeated retry attempts"
    assert all('Retrying' in error for error in driver._logger.errors)
