"""PortHandler that drains host echo after every write (single-wire UART)."""

from dynamixel_sdk import PortHandler

_ECHO_DRAIN_TIMEOUT_S = 0.1


class EchoDrainingPortHandler(PortHandler):
    def writePort(self, packet):
        # Set the blocking timeout BEFORE write: changing timeout from 0 to N
        # between write and read causes the macOS CDC driver to return after
        # the first byte arrives, even though we asked for `n`.
        saved_timeout = self.ser.timeout
        self.ser.timeout = _ECHO_DRAIN_TIMEOUT_S
        try:
            n = super().writePort(packet)
            if n <= 0:
                return n
            drained = self.ser.read(n)
        finally:
            self.ser.timeout = saved_timeout
        if len(drained) != n:
            print(
                f"[echo-drain] WARN: expected {n} echo bytes, got {len(drained)} "
                f"in {_ECHO_DRAIN_TIMEOUT_S}s — bus contention or wrong baud?"
            )
        return n
