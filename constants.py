import graph as gr


####################################################
################# GLOBAL CONSTANTS #################
####################################################
# Inputs
inputType = "file"
if (inputType == "live"):
    import Spatial_simple_cl as spatialC
#fileLocale = "USB-40.txt"
fileLocale = "orientation-one-resampled-full.txt"
#fileLocale = "orientation-one-clean.txt"
#fileLocale = "upndown-resampled.txt"
#fileLocale = "z-pattern-resampled.txt"
#fileLocale = "swinging-test-resampled.txt"
fileLocale = "upndown-resampled-short.txt"
#fileLocale = "try2-resampled.txt"

# Graphs
graphWindow = gr.newWindow("Graphs", 640, 480)
graphChart = gr.newGraph(graphWindow, "Test")
graphAccX = gr.newPlot(graphChart, [], [], 'r', None, None, 3, 'o')
graphAccY = gr.newPlot(graphChart, [], [], 'g', None, None, 3, 'o')
graphAccZ = gr.newPlot(graphChart, [], [], 'b', None, None, 3, 'o')

# House keeping
sleepTime = 0.0001
updateEvery = 10
startTime = 0.0
numSamplesMax = 100

# Complementary filter
beta = 0.25


####################################################
############## CALIBRATION CONSTANTS ###############
####################################################
calGyroOffset  = [-0.20854434285714282, -0.12702977142857147, -0.2672430000000005]
calAccelOffset = [ 0.01725754327857845, -0.00160451736868370,  0.0145056265099980]
calAccelScale  = [ 0.99893057626429155,  1.00137490276358830,  1.0021011922022352]