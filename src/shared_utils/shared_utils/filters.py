"""Streaming signal filters."""
from scipy.signal import butter, sosfilt, sosfilt_zi


class LiveLowPass:
    """Butterworth low-pass applied one sample at a time (keeps its state
    between calls). cutoff and fs in Hz."""

    def __init__(self, cutoff: float, fs: float, order: int = 5):
        # SOS form for numerical stability when streaming.
        self.sos = butter(order, cutoff / (0.5 * fs), btype='low', output='sos')
        self.zi = sosfilt_zi(self.sos)
        self._initialized = False

    def apply(self, sample: float) -> float:
        x = float(sample)
        if not self._initialized:
            # Start at steady state on the first sample instead of ramping from 0.
            self.zi *= x
            self._initialized = True
        y, self.zi = sosfilt(self.sos, [x], zi=self.zi)
        return float(y[0])
