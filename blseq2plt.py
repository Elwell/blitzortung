#!/usr/bin/python

# blseq2plt.py - Generate plot output from logfile
# Andrew Elwell <Andrew.Elwell@gmail.com>
# Licenced under GPL2+

# See the TOA_Blitzortung.pdf (http://blitzortung.org) and
# https://sites.google.com/site/blitzgraphs/ for background
# into the data format.

import struct
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

logfile = open("sample.dat")
try:
	for line in logfile:
		line = line.rstrip()
		# we should really check that ident == '$BLSEQ'
		(ident, timestamp, data) = line.split(',')
		(data,checksum) = data.split('*')
		# split data string into chunks for each pair of readings
		# if using 2 channel firmware:
		chan1 = []
		chan2 = []
		readings = [data[k:k+4] for k in xrange(0,len(data),4)]
		for v in readings:
			c1,c2 = struct.unpack('B B',v.decode('hex'))
			chan1.append(c1)
			chan2.append(c2)
		fig = plt.figure()
		plt.plot(chan1, label='Channel 1')
		plt.plot(chan2, label='Channel 2')
		plt.title(timestamp)
		plt.legend()
		#plt.show()
		fig.savefig(timestamp + '.png')

			
finally:
	logfile.close()

