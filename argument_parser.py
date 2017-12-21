import argparse, sys

class SmartFormatter(argparse.HelpFormatter):

    def _split_lines(self, text, width):
        if text.startswith('R|'):
            return text[2:].splitlines()
        # this is the RawTextHelpFormatter._split_lines
        return argparse.HelpFormatter._split_lines(self, text, width)

class commandline_argument_parser(object):

    class endTimeAction(argparse.Action):
        def __call__(self, parser, namespace, values, option_string=None):
            if values < namespace.startTime:
                print("End time cannot be lower than start time. Setting end time to start time...")
                print "Start time = ", namespace.startTime
                values = namespace.startTime
                print "End time = ", values
                print "It is recommended that you re-run this script with correct start and end times. Calibration data generated on this run will be useless"
                chr = sys.stdin.read(1)
            if values < (namespace.startTime + 3):
                duration = values - namespace.startTime
                if duration == 1:
                    print "You have requested a calibration duration of", duration, "second"
                else:
                    print "You have requested a calibration duration of",duration,"seconds"
                print "It is recommended that you re-run this script with a larger duration. Calibration data generated on this run may not be very useful"
                print "Press enter to continue, or ctrl+c to quit"
                chr = sys.stdin.read(1)
            setattr(namespace, self.dest, values)


    class startTimeAction(argparse.Action):
        def __call__(self, parser, namespace, values, option_string=None):
            if values < 4:
                if values == 1:
                    print "You have requested calibration to start at", values, "second"
                else:
                    print "You have requested calibration to start at", values, "seconds"
                print "It is recommended that you choose a start time of at least four seconds. Calibration data generated on this run will be inaccurate"
                print "Press enter to continue, or ctrl+c to quit"
                chr = sys.stdin.read(1)
            setattr(namespace, self.dest, values)

    def __init__(self):
        self.parser = argparse.ArgumentParser(description='IMU filtering program for ROCO503. All command-line arguments are optional.', formatter_class=SmartFormatter)

        self.parser.add_argument('-l', '--live', metavar='Data source', action='store_const', const="live", default='file', dest='dataSource',
                            help='R|call this flag to tell %(prog)s to use live data \ninstead of a file\ndefault=%(default)s')
        self.parser.add_argument('-c', '--calibrate', action='store_const', const=True, default=False, dest='calibrate',
                            help='call this flag to perform calibration of the IMU\ndefault=%(default)s')
        self.parser.add_argument('-s', '--start', nargs='?', type=int, action=self.startTimeAction, default=4, dest='startTime')
        self.parser.add_argument('-e', '--end', nargs='?', type=int, action=self.endTimeAction, default=10, dest='endTime')

        self.parser.add_argument('-g', '--graph', metavar='\b', dest='triplet', default='0', type=int, action='store',
                help="R|    choose what data you would like to graph:\n"
                "acceleration = 0\n"
                "velocity = 1\n"
                "position = 2\n"
                "angular velocity = 3\n"
                "angular position = 4\n"
                "quaternion = 5\ndefault=%(default)s")

        self.parser.add_argument('-f', '--file', nargs='?', default="IMU_Stationary.txt", dest='fileLocation',
                                 help="R|use this to set the file location, either by \nspecifying a local file" 
                                      "(e.g. data.txt) or a full path \n(e.g. /home/user/data.txt)\n(not tested"
                                      "on windows yet) \ndefault=%(default)s'")



"""
data needed:
    live or not?                    ###
    thing to graph?                 ###
    data file location (optional)   ###
    calibrate vel (seperate file?)  ###
    calibrate accel (seperate file?)#
"""