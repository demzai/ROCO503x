from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg


win = pg.GraphicsWindow(title="-------")
# Set the window size(x,y) [pixels]
win.resize(1000,600)
# Set the window name - the very top bar with the close button
win.setWindowTitle('pyqtgraph example: Scheming >=D')

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)


# Add a plot without any data within the precreated window
# Add a title to the plot
p2 = win.addPlot(title="Multiple curves")
# Turn a background grid on for the graph
p2.showGrid(x=True, y=True)
# Label the x & y axes
p2.setLabel('left', "Y Axis", units='Yo Mama')
p2.setLabel('bottom', "X Axis", units='t')
# State whether the axes are linear of logarithmic
p2.setLogMode(x=True, y=False)
# Add data to the plot
p2.plot(np.random.normal(size=100), pen=(255,0,0), name="Red curve")
p2.plot(np.random.normal(size=10)+5, pen=(0,255,0), name="Green curve")
p2.plot(np.random.normal(size=100)+10,  # Some random data
        pen=(0,0,255),                  # Graph colour
        name="Blue curve",              # Legend name
        symbol='o',                     # Symbol shape
        symbolBrush=(255,0,0),          # Data point dot colour
        symbolPen=None,                 # Data point dot outline colour
        symbolSize=3)                   # Data point dot size


p6 = win.addPlot(title="Updating plot")
# Hide the x-axis
p6.showAxis('bottom', False)
# create a pointer to the data to be plotted
curve = p6.plot(pen='y',                # Line colour
                fillLevel=0,            # Imaginary line (y=0 in this case) to fill up /down to data
                brush=(0,255,255,100))  # Fill colour (R,G,B,A)
data = np.random.normal(size=(10,100))
ptr = 0
def update():
    global curve, data, ptr, p6
    curve.setData( data[ptr] + np.sin( np.linspace(0, 10, 100) )*5 )
    if ptr == 0:
        p6.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
    ptr = (ptr+1)%10
# Create an object to call the update function
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)


# Add a new row of graphs to the window
win.nextRow()

# Cool, but look into later
x2 = np.linspace(-100, 100, 1000)
data2 = np.sin(x2) / x2
p8 = win.addPlot(title="Region Selection")
p8.plot(data2, pen=(255,255,255,200))
lr = pg.LinearRegionItem([400,700])
lr.setZValue(-10)
p8.addItem(lr)

p9 = win.addPlot(title="Zoom on selected region")
p9.plot(data2)
def updatePlot():
    p9.setXRange(*lr.getRegion(), padding=0)
def updateRegion():
    lr.setRegion(p9.getViewBox().viewRange()[0])
lr.sigRegionChanged.connect(updatePlot)
p9.sigXRangeChanged.connect(updateRegion)
updatePlot()

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

