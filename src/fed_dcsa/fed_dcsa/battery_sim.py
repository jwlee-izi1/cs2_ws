"""Simulated battery state tracking for Gazebo drones.

Gazebo does not model battery, so we track energy state in software.
On swap, the new active drone starts with a full battery (E_max) since the
standby was pre-charged (paper Section V-F-b).
"""


class BatterySimulator:
    """Track per-station battery state-of-charge."""

    def __init__(self, n_stations, E_max, E_thr):
        """
        Args:
            n_stations: number of surveillance stations
            E_max: full battery capacity (Wh)
            E_thr: safety reserve threshold (Wh)
        """
        self.n_stations = n_stations
        self.E_max = E_max
        self.E_thr = E_thr
        # All active drones start with full battery
        self.batteries = [E_max] * n_stations

    def get_battery(self, station_id):
        """Return current battery level for a station's active drone."""
        return self.batteries[station_id]

    def deplete(self, station_id, energy_used):
        """Deplete battery for active drone at station.

        Args:
            station_id: which station
            energy_used: Wh consumed this epoch (typically e_i for full activity)
        """
        self.batteries[station_id] = max(0.0, self.batteries[station_id] - energy_used)

    def reset_to_full(self, station_id):
        """Reset battery to E_max after a swap (standby was pre-charged)."""
        self.batteries[station_id] = self.E_max

    def needs_safety_swap(self, station_id):
        """Check if battery is below safety threshold.

        Paper Section V-F-c: E_thr must allow at least one more full epoch.
        """
        return self.batteries[station_id] <= self.E_thr
