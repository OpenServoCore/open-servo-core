"""PortHandler that drains host echo after every write (single-wire UART)."""

from dynamixel_sdk import PortHandler

_ECHO_DRAIN_TIMEOUT_S = 0.1


class EchoDrainingPortHandler(PortHandler):
    def writePort(self, packet):
        n = super().writePort(packet)
        if n <= 0:
            return n
        saved_timeout = self.ser.timeout
        self.ser.timeout = _ECHO_DRAIN_TIMEOUT_S
        try:
            drained = self.ser.read(n)
        finally:
            self.ser.timeout = saved_timeout
        if len(drained) != n:
            print(
                f"[echo-drain] WARN: expected {n} echo bytes, got {len(drained)} "
                f"in {_ECHO_DRAIN_TIMEOUT_S}s — bus contention or wrong baud?"
            )
        return n
