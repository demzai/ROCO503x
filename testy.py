from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
import graph as gr


# Setup the window
win = gr.newWindow("---",1000,600)


# Create a new graph for data to be plotted onto
p2 = gr.newGraph(win, "Multiple curves", "X Axis", 't', "Y Axis", 'Yo Mama',
            True, False, True, False)


# Add data to the plot
gr.newPlot(p2, np.random.normal(size=100), [], (255,0,0))
gr.newPlot(p2, np.random.normal(size=10)+5, [], (0,255,0))
gr.newPlot(p2, np.random.normal(size=100)+10, [], (0,0,255),(255,0,0),None,3,'o')


p6 = gr.newGraph(win, "Updating plot")
# Hide the x-axis
p6.showAxis('bottom', False)
# create a pointer to the data to be plotted
curve = gr.newPlot(p6, [], [], 'y', 'b')
# curve = p6.plot(pen='y',                # Line colour
#                 fillLevel=0,            # Imaginary line (y=0 in this case) to fill up /down to data
#                 brush=(0,255,255,100))  # Fill colour (R,G,B,A)
data = np.random.normal(size=(10,100))
ptr = 0
def update():
    global curve, data, ptr, p6
    gr.updatePlot(curve, data[ptr] + np.sin( np.linspace(0, 10, 100) )*5)
    if ptr == 0:
        p6.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
    ptr = (ptr+1)%10
# Create an object to call the update function
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)




## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

