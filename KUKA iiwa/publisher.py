#!/usr/bin/env python
import rospy
import numpy as np
import scipy.io.wavfile as wav
import wave
import scipy
from scipy import signal, fftpack
from iiwa_msgs.msg import JointPosition
from iiwa_msgs.msg import JointQuantity
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray

def decode(in_data, channels):
	result = np.fromstring(in_data, dtype=np.int16)
	chunk_length = len(result) / channels
	result = np.reshape(result, (chunk_length, channels))
	return result

def gcc_phat(sign, signref):
	# Ensuring minimum FFT length equal to len(sig) + len(refsig)
	n = sign.shape[0] + signref.shape[0]

	# Performing Generalized Cross Correlation Phase Transform
	signPhat = np.fft.rfft(sign, n=n)
	signrefPhat = np.fft.rfft(signref, n=n)
	R = signPhat * np.conj(signrefPhat)
	crossResult = np.fft.irfft(R / np.abs(R), n=(16 * n))

	maxShift = int(16 * n / 2)
	crossResult = np.concatenate((crossResult[-maxShift:], crossResult[:maxShift+1]))

	# Finding shift with maximum similarity value
	resultShift = np.argmax(np.abs(crossResult)) - maxShift
	delay = resultShift / float(16)

	return delay, crossResult

def runMLAT():
	# Initialising objects used to communicate coordinates to KUKA iiwa controller
	pub = rospy.Publisher('position', Float64MultiArray, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	coordinates = Float64MultiArray()
	coordinates.data = []

	count = 0
	vSound = 343.0 # Speed of sound
	w, h = 2, 4; # Number of coordinates, number of sensors
	sepDist = 1.0 # Distance between sensors
	senPos = [[0.0 for x in range(w)] for y in range(h)] # Sensors' position coordinates
	zCoord = 0.008 # Z coordinate

	# First sensor
	senPos[0][0] = 0.0 #x
	senPos[0][1] = 0.0 #y
	senPos[0][2] = zCoord #z

	# Second sensor
	senPos[1][0] = 0.0 #x
	senPos[1][1] = sepDist #y
	senPos[1][2] = zCoord #z

	# Third sensor
	senPos[2][0] = sepDist #x
	senPos[2][1] = 0.0 #y
	senPos[2][2] = zCoord #z

	# Fourth sensor
	senPos[3][0] = sepDist #x
	senPos[3][1] = sepDist #y
	senPos[3][2] = zCoord #z

	# Recording information
	FORMAT = pyaudio.paInt16
	CHANNELS = 4
	RATE = 48000
	CHUNK = 1024
	RECORD_SECONDS = 0.022

	audio = pyaudio.PyAudio()
	signal = [0.0,0.0,0.0,0.0] # Sound signals
	crossResult = [0.0,0.0,0.0,0.0] # Signals similarity - cross-correlation results
	delay = [0.0,0.0,0.0,0.0] # Signal delays calculated by cross-correlation
	tdoa = [0.0,0.0,0.0,0.0] # Time Differences of Arrival
	senOrgDist = [0.0,0.0,0.0,0.0] # Distance from sensor to origin of coordinate system

	# start Recording
	stream = audio.open(format=FORMAT, channels=CHANNELS,
	                rate=RATE, input=True,
	                input_device_index = 7,
	                frames_per_buffer=CHUNK)

	print "recording..."

	# Constantly passing sound source coordinates when connected to KUKA iiwa
	while not rospy.is_shutdown():
		# Record samples from microphones
		recordSamp = []

		# Record a short testing fragment
	    for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
	        data = stream.read(CHUNK)
	        recordSamp.append(data)
		# Join the samples to continuous recording and convert to int
	    recordJoined = ''.join(recordSamp)
	    recordConv = np.fromstring(recordJoined, np.int16)
		# Find maximum amplitude of the testing fragment
	    amplitude = max(recordConv)

		""" If the amplitude of the testing fragment is low it means that no
		significant sound is emitted and the recording is scrapped.
		If the amplitude is above the threshold, it means some sound was
		made in this testing record so it's kept and additional 0.5 s
		is recorded. Then, the sound localisation is performed and
		coordinates are send to KUKA iiwa controller """

	    if amplitude > 3500:
			# Recording additional 0.5s fragment
	        for i in range(0, int(RATE / CHUNK * 0.5)):
	            data = stream.read(CHUNK)
	            recordSamp.append(data)
	        recordJoined = ''.join(recordSamp)

	        for m in range(0,4):
				# Dividing the recording into separate signals from each channel
	            signals = decode(recordJoined, CHANNELS)
	            signal[m] = signals[:,m]

				# Performing cross-correlation in reference to microphone number 1
	            delay[m], crossResult[m] = gcc_phat(signal[m], signal[0])

				# Converting delay value from number of samples to seconds
	            tdoa[m] = float(delay[m])/float(RATE)

				# Calculating distance between sensor that recorded given signal to origin of coordinate system
	            senOrgDist[m]=(((senPos[m][0])**2.0 + (senPos[m][1])**2.0 + (senPos[m][2])**2.0)**(0.5))

	        # Recalculating TDOA by moving starting point to the smallest (or most negative) TDOA value
			mostNeg = min(tdoa)
			tdoa[:] = [x - mostNeg for x in tdoa]

			# Labeling sensors according to the delay in recorded signal
			tdoaSort = sorted(tdoa)
			senNum1 = tdoa.index(tdoaSort[1])
			senNum2 = tdoa.index(tdoaSort[2])
			senNum3 = tdoa.index(tdoaSort[3])

			# Coordinates of sensor with the smallest non-zero delay
			x1 = senPos[senNum1][0]
			y1 = senPos[senNum1][1]

			# Constructing MLAT matrix
			A = []
			for m in [senNum2,senNum3]:
			    x = senPos[m][0]
			    y = senPos[m][1]
			    Rm1 = tdoa[m]*vSound
			    R21 = tdoaSort[1]*vSound
			    Am = -(x*R21 - x1*Rm1)
			    Bm = -(y*R21 - y1*Rm1)
			    Dm = ((senOrgDist[m]**2.0)*R21 - (senOrgDist[senNum1]**2.0)*Rm1 - Rm1*R21*(Rm1-R21))*0.5
			    A += [[Am,Bm,Dm]]

			# Solving MLAT matrix using SVD
			A = np.array(A)
			(_,_,v) = np.linalg.svd(A)
			# Getting the minimizer
			w = v[2,:]
			w /= w[2]

			# Publishing the sound source coordinates to listener node
			coordinates.data = [w[0],w[1],zCoord]
			rospy.loginfo(coordinates)
			pub.publish(coordinates)
			count += 1

if __name__ == '__main__':
	try:
		runMLAT()
	except rospy.ROSInterruptException:
		pass
