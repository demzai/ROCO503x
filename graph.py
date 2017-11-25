import numpy as np
import pyqtgraph as pg


# Create a new window to store graphs in
def newWindow(titleText, xSize, ySize):
    # Create the window
    win = pg.GraphicsWindow(title=titleText)
    # Ensure the title is set
    win.setWindowTitle(titleText)
    # Preset the window size
    win.resize(xSize, ySize)
    # Enable antialiasing for prettier plots
    pg.setConfigOptions(antialias=True)
    # Return the window
    return win


# Create a new graph within a given window
def newGraph(window, plotTitle=None, xLabel=None, xUnits=None, yLabel=None, yUnits=None,
             xLog=False, yLog=False, showGrid=True, newRow=False):
    # Create a new graph, also creating a new row if so desired
    if (newRow == True):
        window.nextRow()
    graph = window.addPlot(title=plotTitle)

    # Setup grid & labels
    graph.showGrid(x=showGrid, y=showGrid)
    graph.setLogMode(x=xLog, y=yLog)
    graph.setLabel('bottom', xLabel, units=xUnits)
    graph.setLabel('left'  , yLabel, units=yUnits)

    # Return the graph handle
    return graph


# Add a new plot into a given graph
def newPlot(graph, yData=[], xData=[],
            lineColour='w', markInnerColour=None, markOuterColour=None,
            markSize=3, markShape='o'):

    # Resolve the no-xData condition
    if (yData == []):
        plt = graph.plot(pen=lineColour,                # Graph line colour
                         symbolBrush=markInnerColour,   # Data point dot colour
                         symbolPen=markOuterColour,     # Data point dot outline colour
                         symbolSize=markSize,           # Data point dot size
                         symbol=markShape)              # Data point dot shape
    elif (xData == []):
        plt = graph.plot(np.array(yData),               # Y-axis data
                         pen=lineColour,                # Graph line colour
                         symbolBrush=markInnerColour,   # Data point dot colour
                         symbolPen=markOuterColour,     # Data point dot outline colour
                         symbolSize=markSize,           # Data point dot size
                         symbol=markShape)              # Data point dot shape
    else:
        plt = graph.plot(np.array(xData),               # X-axis data
                         np.array(yData),               # Y-axis data
                         pen=lineColour,                # Graph line colour
                         symbolBrush=markInnerColour,   # Data point dot colour
                         symbolPen=markOuterColour,     # Data point dot outline colour
                         symbolSize=markSize,           # Data point dot size
                         symbol=markShape)              # Data point dot shape

    # Return the plot data handle
    return plt


# Update the data used in a given plot
def updatePlot(plotHandle, yData, xData=[]):
    if (xData == []):
        plotHandle.setData(np.array(yData))
    else:
        plotHandle.setData(np.array(xData), np.array(yData))
    return










