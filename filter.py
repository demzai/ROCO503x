import numpy as np
from scipy import signal

# @todo create lowpass and high pass filters
# @todo create complementary filter

# IMU sampling frequency is 62.5Hz
# Reference: https://stackoverflow.com/a/17884962/3303504
def getFilter(filterOrder, cutoffFrequency, samplingFrequency):
    return signal.firwin(
        numtaps=filterOrder,
        cutoff=cutoffFrequency,
        nyq=samplingFrequency/2)


def runLowPassFilter(filter, data):
    return signal.lfilter(filter, [1.0], data)


if __name__ == '__main__':
    data = np.sin(np.linspace(0.0, 2.0, 125) * 2*np.pi * 30) * 10
    print(np.max(data))

    filter = getFilter(10, 10, 62.5)
    newData = runLowPassFilter(filter, data)
    print(np.max(newData))