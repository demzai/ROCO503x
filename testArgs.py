import sys
import argument_parser

clp = argument_parser.commandline_argument_parser()
args = clp.parser.parse_args()
print args.dataSource
print args.triplet
print args.fileLocation
print args.calibrate


import sys, signal

print "f"
#print args.endTime