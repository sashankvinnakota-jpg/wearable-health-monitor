#!/usr/bin/env python3
"""
capture_and_plot.py — capture and validate wearable-health-monitor telemetry.

Two jobs:
  1. Read '$I' / '$P' CSV lines off the ESP32 serial port into a file.
  2. Analyse that file: sample-rate check, noise floor, and a heart-rate
     estimate from the pulse waveform's frequency spectrum.

Usage
-----
  # capture 30 s and plot
  python capture_and_plot.py --port COM5 --seconds 30

  # re-analyse a capture you already have (no board needed)
  python capture_and_plot.py --replay session.csv

Requires: pyserial, numpy, matplotlib
  pip install pyserial numpy matplotlib
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np

# Physiological plausibility window for a resting-to-active human heart rate.
# 42-180 bpm = 0.7-3.0 Hz. Anything outside this is noise, not a pulse.
HR_MIN_HZ, HR_MAX_HZ = 0.7, 3.0


# ── capture ──────────────────────────────────────────────────────────────────
def capture(port, baud, seconds, outfile):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial not installed.  pip install pyserial")

    print(f"Opening {port} @ {baud}...")
    with serial.Serial(port, baud, timeout=1) as ser:
        # The ESP32 resets when the port opens; wait for it to boot.
        time.sleep(2.0)
        ser.reset_input_buffer()

        print(f"Capturing {seconds} s.  Put a finger on the pulse sensor NOW.")
        deadline = time.time() + seconds
        kept = dropped = 0

        with open(outfile, "w", encoding="utf-8") as f:
            while time.time() < deadline:
                raw = ser.readline()
                if not raw:
                    continue
                try:
                    line = raw.decode("utf-8", errors="strict").strip()
                except UnicodeDecodeError:
                    dropped += 1          # baud mismatch or buffer overrun
                    continue
                if line.startswith("$"):
                    f.write(line + "\n")
                    kept += 1
                    if kept % 200 == 0:
                        remaining = int(deadline - time.time())
                        print(f"  {kept} rows... ({remaining}s left)", end="\r")

    print(f"\nCaptured {kept} rows to {outfile}"
          + (f"  ({dropped} corrupt lines dropped)" if dropped else ""))
    if kept == 0:
        sys.exit("No '$' rows seen. Is CSV_STREAM_ENABLED set to 1 and reflashed?")
    if dropped > kept * 0.01:
        print("WARNING: >1% corrupt lines — raise the baud rate or lower the "
              "sample rate; the serial link is saturated.")
    return outfile


# ── parse ────────────────────────────────────────────────────────────────────
def load(path):
    imu, ppg = [], []
    bad = 0
    for line in Path(path).read_text(encoding="utf-8", errors="ignore").splitlines():
        parts = line.strip().split(",")
        try:
            if parts[0] == "$I" and len(parts) == 10:
                imu.append([float(x) for x in parts[1:]])
            elif parts[0] == "$P" and len(parts) == 5:
                ppg.append([float(x) for x in parts[1:]])
        except ValueError:
            bad += 1
    if bad:
        print(f"  ({bad} malformed rows skipped)")

    imu = np.array(imu) if imu else np.empty((0, 9))
    ppg = np.array(ppg) if ppg else np.empty((0, 4))
    return imu, ppg


# ── analysis ─────────────────────────────────────────────────────────────────
def actual_rate(t_ms):
    """Measured sample rate from timestamps, plus jitter."""
    if len(t_ms) < 2:
        return 0.0, 0.0
    dt = np.diff(t_ms)
    dt = dt[dt > 0]
    if len(dt) == 0:
        return 0.0, 0.0
    return 1000.0 / np.mean(dt), np.std(dt)


def detrend(x, fs, window_s=1.5):
    """Remove the DC bias and slow baseline wander with a moving average.

    The pulse sensor's DC level (~1.55 V) is 100x larger than the pulsatile AC
    component. Without removing it the FFT is one giant spike at 0 Hz and the
    heartbeat is invisible.
    """
    w = max(3, int(fs * window_s) | 1)          # odd-length window
    kernel = np.ones(w) / w
    baseline = np.convolve(x, kernel, mode="same")
    return x - baseline


def estimate_hr(sig, fs, label):
    """Dominant frequency in the plausible HR band -> bpm."""
    if len(sig) < fs * 5:
        print(f"  {label}: too short for a reliable estimate "
              f"({len(sig)} samples, need {int(fs*5)})")
        return None, None, None

    x = detrend(sig.astype(float), fs)
    x *= np.hanning(len(x))                     # reduce spectral leakage

    spec = np.abs(np.fft.rfft(x))
    freqs = np.fft.rfftfreq(len(x), d=1.0 / fs)

    band = (freqs >= HR_MIN_HZ) & (freqs <= HR_MAX_HZ)
    if not band.any():
        return None, None, None

    peak_i = np.argmax(spec[band])
    peak_f = freqs[band][peak_i]
    peak_mag = spec[band][peak_i]

    # Confidence: peak height vs the median of the rest of the band.
    # A real pulse stands well clear; noise gives a ratio near 1.
    noise = np.median(spec[band])
    snr = peak_mag / noise if noise > 0 else 0.0

    bpm = peak_f * 60.0
    verdict = "plausible" if snr > 4 else "WEAK — likely noise, not a pulse"
    print(f"  {label}: {bpm:.1f} bpm  ({peak_f:.2f} Hz, peak/median = {snr:.1f}) "
          f"-> {verdict}")
    return bpm, freqs, spec


def report(imu, ppg):
    print("\n" + "=" * 62)
    print(" CAPTURE ANALYSIS")
    print("=" * 62)

    fs_imu = 0.0
    if len(imu):
        t = imu[:, 1]
        fs_imu, jitter = actual_rate(t)
        dur = (t[-1] - t[0]) / 1000.0
        ax, ay, az = imu[:, 2], imu[:, 3], imu[:, 4]
        mag = np.sqrt(ax**2 + ay**2 + az**2)

        print(f"\nIMU: {len(imu)} samples over {dur:.1f} s")
        print(f"  measured rate     {fs_imu:.1f} Hz   (firmware target 100 Hz)")
        print(f"  interval jitter   {jitter:.2f} ms sd")
        if fs_imu < 90:
            print("  ^ WARNING: running slow. The task is missing its deadline, "
                  "or serial output is blocking it.")

        print(f"  |a| mean          {mag.mean():.4f} g   (gravity = 1.000)")
        print(f"  |a| sd            {mag.std():.4f} g   <- noise floor")
        print(f"  per-axis mean     ax={ax.mean():+.4f}  ay={ay.mean():+.4f}  "
              f"az={az.mean():+.4f}")
        print(f"  gyro bias         gx={imu[:,5].mean():+.2f}  "
              f"gy={imu[:,6].mean():+.2f}  gz={imu[:,7].mean():+.2f} deg/s")

    print("\nHeart rate estimates:")
    if len(imu):
        estimate_hr(imu[:, 8], fs_imu, "analog pulse sensor")
    if len(ppg):
        fs_ppg, _ = actual_rate(ppg[:, 1])
        print(f"  MAX30102: {len(ppg)} samples at {fs_ppg:.1f} Hz, "
              f"red mean={ppg[:,2].mean():.0f}  ir mean={ppg[:,3].mean():.0f}")
        if ppg[:, 3].mean() < 10000:
            print("  ^ IR below 10000: no finger contact, or LED current too low "
                  "(raise MAX_LED_CURRENT_INIT).")
        estimate_hr(ppg[:, 3], fs_ppg, "MAX30102 IR")
    else:
        print("  MAX30102: no samples captured")
    print("=" * 62 + "\n")
    return fs_imu


def plot(imu, ppg, fs_imu):
    import matplotlib.pyplot as plt

    n = 2 + (1 if len(ppg) else 0)
    fig, axes = plt.subplots(n, 1, figsize=(11, 3 * n))
    axes = np.atleast_1d(axes)

    if len(imu):
        t = (imu[:, 1] - imu[0, 1]) / 1000.0
        mag = np.sqrt(imu[:, 2] ** 2 + imu[:, 3] ** 2 + imu[:, 4] ** 2)

        axes[0].plot(t, mag, lw=0.8, label="|a|")
        axes[0].axhline(1.0, ls="--", lw=0.8, color="k", label="1 g")
        axes[0].set_ylabel("accel (g)")
        axes[0].set_title("Accelerometer magnitude")
        axes[0].legend(loc="upper right", fontsize=8)

        pulse = imu[:, 8]
        axes[1].plot(t, detrend(pulse, fs_imu or 100), lw=0.8, color="crimson")
        axes[1].set_ylabel("pulse (ADC, detrended)")
        axes[1].set_title("Analog pulse sensor — look for regular peaks ~1 Hz apart")

    if len(ppg):
        tp = (ppg[:, 1] - ppg[0, 1]) / 1000.0
        axes[2].plot(tp, ppg[:, 3], lw=0.8, label="IR")
        axes[2].plot(tp, ppg[:, 2], lw=0.8, label="Red", alpha=0.7)
        axes[2].set_ylabel("MAX30102 raw")
        axes[2].set_title("PPG channels")
        axes[2].legend(loc="upper right", fontsize=8)

    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()

    out = "capture_plot.png"
    fig.savefig(out, dpi=130)
    print(f"Saved {out}")
    plt.show()


# ── main ─────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200,
                    help="must match SERIAL_BAUD (default 115200)")
    ap.add_argument("--seconds", type=int, default=30, help="capture duration")
    ap.add_argument("--out", default="session.csv", help="output CSV path")
    ap.add_argument("--replay", help="analyse an existing CSV, skip capture")
    args = ap.parse_args()

    if args.replay:
        path = args.replay
    elif args.port:
        path = capture(args.port, args.baud, args.seconds, args.out)
    else:
        ap.error("give --port to capture, or --replay to analyse a saved file")

    imu, ppg = load(path)
    if not len(imu) and not len(ppg):
        sys.exit("No parseable data rows found.")

    fs_imu = report(imu, ppg)

    try:
        plot(imu, ppg, fs_imu)
    except ImportError:
        print("matplotlib not installed — skipping plots.  pip install matplotlib")


if __name__ == "__main__":
    main()
