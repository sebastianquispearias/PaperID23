
"""
Static PoI node: represents a Point of Interest in the simulation.
"""

from gradysim.protocol.interface import IProtocol
from gradysim.protocol.messages.telemetry import Telemetry

from gradysim.protocol.plugin.random_mobility import RandomMobilityPlugin, RandomMobilityConfig

class POIProtocol(IProtocol):
    def initialize(self):
        # … tu init habitual …
        # agrega un plugin de movilidad estática (speed=0)
        self.mob = RandomMobilityPlugin(
            self,
            RandomMobilityConfig(x_range=(0,0), y_range=(0,0), z_range=(0.0,0.0), tolerance=0)
        )
    def handle_telemetry(self, telemetry: Telemetry) -> None:
        pass

    def handle_timer(self, timer: str) -> None:
        pass

    def handle_packet(self, message: str) -> None:
        pass

    def finish(self) -> None:
        pass
