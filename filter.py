####################################################
################### DEPENDENCIES ###################
####################################################
from scipy.signal import butter, cheby1, cheby2, lfilter
from helper_functions import *


####################################################
################# GLOBAL CONSTANTS #################
####################################################
imuSampleFrequency = 137
chebyRipple = 1 #dB


# IMU sampling frequency is 62.5Hz
# Create a butterworth filter & apply it to provided data
def butter_filter(data, order, cutoff, type, fSample):
    nyq = 0.5 * fSample
    p = butter(order, cutoff/nyq, btype=type)
    return lfilter(p[0], p[1], data)

def butter_bandpass_filter(data, order, cutoff, type, fSample):
    nyq = 0.5 * fSample
    p = butter(order, cutoff/nyq, btype=type)
    return lfilter(p[0], p[1], data)


# Create a chebyshev top-wobble filter & apply it to provided data
def cheby_top_filter(data, order, cutoff, type, fSample):
    nyq = 0.5 * fSample
    p = cheby1(order, chebyRipple, cutoff/nyq, btype=type)
    return lfilter(p[0], p[1], data)


# Create a chebyshev bottom-wobble filter & apply it to provided data
def cheby_bottom_filter(data, order, cutoff, type, fSample):
    nyq = 0.5 * fSample
    p = cheby2(order, chebyRipple, cutoff/nyq, btype=type)
    return lfilter(p[0], p[1], data)


# Select a filter & perform it
def doFilter(data, filterSelect, filterType, cutoffFrequency, order, sampleFrequency):
    if(filterSelect == 'butter'):
        return butter_filter(data, order, cutoffFrequency,
                             filterType, sampleFrequency)
    elif(filterSelect == 'chebyT'):
        return cheby_top_filter(data, order, cutoffFrequency,
                                filterType, sampleFrequency)
    elif(filterSelect == 'chebyB'):
        return cheby_bottom_filter(data, order, cutoffFrequency,
                                   filterType, sampleFrequency)
    else:
        return [0]


# Filter a data set
# filterX = [filterSelect, filterType, cutoffFrequency(ies), order]
def filterData(data, columns, filter, sampleFrequency=imuSampleFrequency):
    numColumns = len(columns)
    returnVal = []
    for column in range(0, numColumns):
        returnVal += [doFilter(getCol(data, columns[column]), filter[0],
                               filter[1], np.array(filter[2]),
                               filter[3], sampleFrequency)[-1]]
    return returnVal

