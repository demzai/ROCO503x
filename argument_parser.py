import argparse

class SmartFormatter(argparse.HelpFormatter):

    def _split_lines(self, text, width):
        if text.startswith('R|'):
            return text[2:].splitlines()
        # this is the RawTextHelpFormatter._split_lines
        return argparse.HelpFormatter._split_lines(self, text, width)


class commandline_argument_parser(object):

    def __init__(self):
        self.parser = argparse.ArgumentParser(description='IMU filtering program for ROCO503', formatter_class=SmartFormatter)

        self.parser.add_argument('-l', '--live', metavar='Data source', action='store_const', const="live", default='file', dest='dataSource',
                            help='Tells %(prog)s whether to use live data or read from a file')
        self.parser.add_argument('-c', '--calibrate', action='store_const', const=True, default=False, dest='calibrate',
                            help='Call this flag to perform calibration of the IMU')

        self.parser.add_argument('-g', '--graph', metavar='\b', dest='triplet', default='0', type=int, action='store',#, default='1', const='1',
                help="R|    Choose what data you would like to graph:\n"
                "Acceleration = 0\n"
                "velocity = 1\n"
                "position = 2\n"
                "angular velocity = 3\n"
                "angular position = 4\n"
                "quaternion = 5")

        #self.parser.add_argument('-f', '--file', nargs='?', type=argparse.FileType('r'), default="IMU_Stationary.txt", dest='fileLocation')
        self.parser.add_argument('-f', '--file', nargs='?', default="IMU_Stationary.txt", dest='fileLocation')




"""
data needed:
    live or not?                    ###
    thing to graph?                 ###
    data file location (optional)   ###
    calibrate (seperate file?)      # #
"""