import matplotlib.pyplot as plt
import numpy as np
import wave
import scipy
from scipy import signal, fftpack
import scipy.io.wavfile as wav

vSound = 343.0 # Speed of sound
w, h = 2, 4; # Number of coordinates, number of sensors
sepDist = 1.0 # Distance between sensors
senPos = [[0.0 for x in range(w)] for y in range(h)] # Sensors' position coordinates

# First sensor
senPos[0][0] = 0.0 #x
senPos[0][1] = 0.0 #y

# Second sensor
senPos[1][0] = 0.0 #x
senPos[1][1] = sepDist #y

# Third sensor
senPos[2][0] = sepDist #x
senPos[2][1] = 0.0 #y

# Fourth sensor
senPos[3][0] = sepDist #x
senPos[3][1] = sepDist #y


signal = [0.0,0.0,0.0,0.0] # Sound signals
crossResult = [0.0,0.0,0.0,0.0] # Signals similarity - cross-correlation results
delay = [0.0,0.0,0.0,0.0] # Signal delays calculated by cross-correlation
tdoa = [0.0,0.0,0.0,0.0] # Time Differences of Arrival
senOrgDist = [0.0,0.0,0.0,0.0] # Distance from sensor to origin of coordinate system

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

for m in range(0,4):
    # Importing four wave files named "test0.wav", "test1.wav", etc. and records their sampling rate
    fileName = 'test%s.wav' % m
    rate, signal[m] = wav.read(fileName)

    # Performing cross-correlation in reference to "test0.wav"
    delay[m], crossResult[m] = gcc_phat(signal[m], signal[0])

    # Converting delay value from number of samples to seconds
    tdoa[m] = float(delay[m])/float(rate)

    # Calculating distance between sensor that recorded given signal to origin of coordinate system
    senOrgDist[m]=(((senPos[m][0])**2.0 + (senPos[m][1])**2.0)**(0.5))

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

# Printing the coordinates of localised sound source
print "x coordinate: ", w[0]
print "y coordinate: ", w[1]

# Plotting the sound wave and cross-correlation similarity graph
soundAxis = np.linspace(0, len(signal[0])/float(rate), num=len(signal[0]))
crossAxis = np.linspace(-len(crossResult[0])/float(rate), len(crossResult[0])/float(rate), num=len(crossResult[0]))

plt.figure(1)
plt.subplot(511)
plt.title('Signal Wave')
for m in range(0,4):
    plt.plot(soundAxis,signal[m])
for m in range(0,4):
    plt.subplot(512+m)
    plt.plot(crossAxis,crossResult[m])

plt.show()
