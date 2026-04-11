"""Incremental CSV logging for Fed-DCSA experiments.

Writes three CSV files:
- rounds.csv:     one row per algorithm round (gate, activity levels, actions)
- batteries.csv:  one row per station per round (battery state)
- swaps.csv:      one row per swap event
"""

import csv
import os


class CSVLogger:
    """Incremental CSV logger that writes after every round."""

    def __init__(self, log_dir, experiment_name, n_stations):
        self.n_stations = n_stations
        os.makedirs(log_dir, exist_ok=True)

        prefix = os.path.join(log_dir, experiment_name)

        # Rounds CSV
        self._rounds_path = f'{prefix}_rounds.csv'
        self._rounds_file = open(self._rounds_path, 'w', newline='')
        self._rounds_writer = csv.writer(self._rounds_file)
        rounds_header = ['k', 'b_k', 'G_tilde', 'eta_k', 'r_drift', 'gamma']
        for i in range(n_stations):
            rounds_header.extend([f'x_before_{i}', f'x_after_{i}', f'a_{i}'])
        self._rounds_writer.writerow(rounds_header)

        # Batteries CSV
        self._batt_path = f'{prefix}_batteries.csv'
        self._batt_file = open(self._batt_path, 'w', newline='')
        self._batt_writer = csv.writer(self._batt_file)
        self._batt_writer.writerow([
            'k', 'station_id', 'active_drone', 'battery_wh',
        ])

        # Swaps CSV
        self._swaps_path = f'{prefix}_swaps.csv'
        self._swaps_file = open(self._swaps_path, 'w', newline='')
        self._swaps_writer = csv.writer(self._swaps_file)
        self._swaps_writer.writerow([
            'k', 'station_id', 'old_active', 'new_active', 'battery_at_swap',
        ])

    def log_round(self, result, stations):
        """Log one round's data.

        Args:
            result: RoundResult from algorithm.py
            stations: list of StationState (after round processing)
        """
        row = [
            result.k, result.b_k,
            f'{result.G_tilde:.6f}', f'{result.eta_k:.6f}',
            f'{result.r_drift:.6f}', f'{result.gamma:.8f}',
        ]
        for i in range(self.n_stations):
            row.extend([
                f'{result.x_before[i]:.6f}',
                f'{result.x_after[i]:.6f}',
                result.actions[i],
            ])
        self._rounds_writer.writerow(row)
        self._rounds_file.flush()

        # Battery log
        for i, s in enumerate(stations):
            self._batt_writer.writerow([
                result.k, i, s.active_drone, f'{s.battery:.2f}',
            ])
        self._batt_file.flush()

    def log_swap(self, k, station_id, old_active, new_active, battery_at_swap):
        """Log a swap event."""
        self._swaps_writer.writerow([
            k, station_id, old_active, new_active, f'{battery_at_swap:.2f}',
        ])
        self._swaps_file.flush()

    def close(self):
        """Flush and close all CSV files."""
        for f in [self._rounds_file, self._batt_file, self._swaps_file]:
            f.flush()
            f.close()

    @property
    def rounds_path(self):
        return self._rounds_path

    @property
    def batteries_path(self):
        return self._batt_path

    @property
    def swaps_path(self):
        return self._swaps_path
