#!/usr/bin/python

# blseq2plt.py - Generate gnuplot output from logfile
# Andrew Elwell <Andrew.Elwell@gmail.com>
# Licenced under GPL2+

# See the TOA_Blitzortung.pdf (http://blitzortung.org) and
# https://sites.google.com/site/blitzgraphs/ for background
# into the data format.
#### This version process all the $BLSEQ records form the sample.dat file
####  and save the the graphs


import struct
import matplotlib.pyplot as plt

logfile = open("sample.dat")
try:
	
	for line in logfile:
		line = line.rstrip()
		(ident, timestamp, data) = line.split(',')
		(data,checksum) = data.split('*')
		# split data string into chunks for each pair of readings
		# if using 2 channel firmware:
		chan1 = []
		chan2 = []
		readings = [data[k:k+4] for k in xrange(0,len(data),4)]
		plot = []
		for v in readings:
			c1,c2 = struct.unpack('B B',v.decode('hex'))
			chan1.append(c1)
			chan2.append(c2)

		plt.plot(chan1, label='Channel 1')
		plt.plot(chan2, label='Channel 2')
		plt.title(timestamp)
## This legened() command draw the legends. If you want more graphs in a pic, this legends hide parts of the graph.
## Comment out, if you dont need legends
##		plt.legend()
## This show() command enable to display the graph on the screen. If you enable it, you can see the graphs on the 
## sceen only
##		plt.show()
## This savefig(fn) command save the generated graphs in separate files. Use it with the next, clf() cpommand, to 
## make each graphs in separated files
		plt.savefig(timestamp)
## If you leave this clf() command commented out, all the data will put increasengly into the generated files.
##		plt.clf()
finally:
		logfile.close()
