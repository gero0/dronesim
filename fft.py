from scipy.fft import fft, fftfreq, ifft

import numpy as np
import csv
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal

class LPF:
    def __init__(self, cutoff, sampling):
        alpha = 6.28318530718 * cutoff/ sampling
        self.prev = 0.0
        self.a = alpha / (1.0 + alpha)
        self.b = 1.0 / (1.0 + alpha)

    def update(self, sample):
        output = self.a * sample + self.b * self.prev
        self.prev = sample
        return output
    
    def process_block(self, input_list):
        output_list = [self.update(x) for x in input_list]
        return output_list
    
def resample_by_interpolation(signal, input_fs, output_fs):

    scale = output_fs / input_fs
    # calculate new length of sample
    n = round(len(signal) * scale)

    # use linear interpolation
    # endpoint keyword means than linspace doesn't go all the way to 1.0
    # If it did, there are some off-by-one errors
    # e.g. scale=2.0, [1,2,3] should go to [1,1.5,2,2.5,3,3]
    # but with endpoint=True, we get [1,1.4,1.8,2.2,2.6,3]
    # Both are OK, but since resampling will often involve
    # exact ratios (i.e. for 44100 to 22050 or vice versa)
    # using endpoint=False gets less noise in the resampled sound
    resampled_signal = np.interp(
        np.linspace(0.0, 1.0, n, endpoint=False),  # where to interpret
        np.linspace(0.0, 1.0, len(signal), endpoint=False),  # known positions
        signal,  # known data points
    )
    return resampled_signal

gyro = pd.read_csv("gyro.csv")
pr = gyro['PR'].tolist()

# gyro = pd.read_csv("acc.csv")
# pr = gyro['X'].tolist()

plt.figure()
plt.plot(pr)

org_fs = 333
target_fs = 200

pr = resample_by_interpolation(pr, org_fs, target_fs)
plt.figure()
plt.plot(pr)

plt.figure()

#FIR
numtaps = 32
fc = 30
fs = target_fs
coeffs = signal.firwin(numtaps, fc, fs=fs)
filtered = signal.lfilter(coeffs, 1.0, pr)
print(coeffs)

w, h = signal.freqz(coeffs, worN=8000)

plt.plot( (w/np.pi) * fs/2, abs(h))


# lpf = LPF(15, fs)
# filtered = lpf.process_block(pr)

# sos = signal.butter(5, 20, 'low', fs=fs, output='sos')
# filtered = signal.sosfilt(sos, pr)


T = 1.0 / fs

fft_pr = fft(pr)

fft_filtered = fft(filtered)

N = len(pr)

xf = fftfreq(N, T)[:N//2]

plt.plot(xf[1:], 2.0/N * np.abs(fft_pr[0:N//2]) [1:])
plt.plot(xf[1:], 2.0/N * np.abs(fft_filtered[0:N//2]) [1:])

plt.figure()

final = ifft(filtered)
plt.plot(filtered)

plt.grid()

plt.show()