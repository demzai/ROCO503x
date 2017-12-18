# @todo create complementary filter

####################################################
################### DEPENDENCIES ###################
####################################################
from scipy.signal import butter, cheby1, cheby2, lfilter
import numpy as np


####################################################
################# GLOBAL CONSTANTS #################
####################################################
imuSampleFrequency = 62.5
chebyRipple = 1 #dB


# IMU sampling frequency is 62.5Hz
# Create a butterworth filter & apply it to provided data
def butter_filter(data, order, cutoff, type, fSample):
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


# Extract a single column of data
def getCol(data, column):
    return np.array(data)[:, column].tolist()


# Filter a data set
# filterX = [filterSelect, filterType, cutoffFrequency(ies), order]
def filterData(data, filter1, filter2, sampleFrequency=imuSampleFrequency):
    returnVal = [data[-1][0], 0,0,0, 0,0,0]

    for x in range(1, 4):
        returnVal[ x ] = doFilter(getCol(data, x  ), filter1[0],
                                  filter1[1], np.array(filter1[2]),
                                  filter1[3], sampleFrequency)[-1]
        returnVal[x+3] = doFilter(getCol(data, x+3), filter2[0],
                                  filter2[1], np.array(filter2[2]),
                                  filter2[3], sampleFrequency)[-1]
    return returnVal


# Filter a specific set of data
def filterSensor(data, filter, sampleFrequency=imuSampleFrequency):
    returnVal = [0,0,0]

    for x in range(1, 4):
        returnVal[ x ] = doFilter(getCol(data, x  ), filter[0],
                                  filter[1], np.array(filter[2]),
                                  filter[3], sampleFrequency)[-1]
    return returnVal






















