import numpy as np
import scipy
import wave
import matplotlib.pyplot as plt
import scipy.io.wavfile as wav
from scipy.io.wavfile import write

def second_smallest(numbers):
    a1, a2 = float('inf'), float('inf')
    for x in numbers:
        if x <= a1:
            a1, a2 = x, a1
        elif x < a2:
            a2 = x
    return a2

minSep = 0.5 # Minimum tested separation between microphones
maxSep = 3.0 # Maximum tested separation between microphones
sampNum = 15 # Number of distances of separation tested
distances = np.linspace(minSep,maxSep,sampNum) # Distances of separation tested

# Collection of errors in x and y direction between actual and calculated emitter position
errXCoord = []
errYCoord = []

for sepDist in distances:
    # Collection of errors at given separation distance
    errAtDistX = []
    errAtDistY = []
    for m in range(0,5):
        rate, sample2 = wav.read('clap.wav') # Importing a sound of clapping
        volume = 0.001 # Volume of sound
        rate = 44100 # Sampling rate, Hz, must be integer
        duration = 0.01 # Duration of generated testing file in seconds

        vSound = 343.0
        w, h = 3, 4;

        emPos = [5.0+np.random.random()/2,5.0+np.random.random()/2,5.0] # Actual emitter position
        senPos = [[0.0 for x in range(w)] for y in range(h)] # Sensors' position coordinates

        zCoord = 0.1
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


        emSenDist = [0.0,0.0,0.0,0.0] # Distances from emitter to sensors
        toa = [0.0,0.0,0.0,0.0] # Times of Arrival from emitter to sensors
        tdoa = [0.0,0.0,0.0,0.0] # Time Differences of Arrival
        senOrgDist = [0.0,0.0,0.0,0.0] # Distance from sensor to origin of coordinate system
        signal = [0.0,0.0,0.0,0.0] # Sound signals
        delay = [0.0,0.0,0.0,0.0] # Signal delays calculated by cross-correlation
        signalNorm = [0.0,0.0,0.0,0.0] # Sound signals after normalisation

        for m in range(0,4):
            # Calculating distances from emitter to each sensor
            emSenDist[m]=(((senPos[m][0] - emPos[0])**2.0 + (senPos[m][1] - emPos[1])**2.0 + (senPos[m][2] - emPos[2])**2.0)**(0.5))

            # Calculating actual time of arrival of signal from emitter to each sensor
            toa[m]=emSenDist[m]/vSound

        toa0 = min(toa) # Minimum Time of Arrival
        emSenDist0 = min(emSenDist) # Minimum distance from emmiter to sensor
        for m in range(0,4):
            # Calculating actual time difference of arrival
            tdoa[m]=toa[m]-toa0

            # Setting origin of the coordinate system
            senOrgDist[m]=emSenDist[m]-emSenDist0

            # Generating artificial audio files simulating recordings from each microphone
            before = int(rate*(duration+tdoa[m]))
            after = int(rate*(duration-tdoa[m]))
            sample1 = np.zeros(before)
            sample3 = np.zeros(after)
            sample = np.append(sample1,volume*sample2)
            sample = np.append(sample,sample3)

            filename = 'test%s.wav' % m
            write(filename, rate, volume*sample)

        # Loading artificially created audio files
        for m in range(0,4):
            filename = 'test%s.wav' % m
            rate, signal[m] = wav.read(filename)

        # Normalising reference audio file
        signalNorm[0] = (signal[0] - np.mean(signal[0])) / (np.std(signal[0]) * len(signal[0]))


        for m in range(0,4):
            # Normalising each audio file with respect to reference audio file
            signalNorm[m] = (signal[m] - np.mean(signal[m])) /  np.std(signal[m])

            # Performing cross-correlation with respect to reference audio file
            delay[m] = np.correlate(signalNorm[0],signalNorm[m],'full')

            # Converting delay value from number of samples to seconds
            tdoa[m] = float(np.argmax(delay[m]) - len(signalNorm[0]))/float(rate)

            # Calculating distance between sensor simulated to origin of coordinate system
            senOrgDist[m]=(((senPos[m][0])**2.0 + (senPos[m][1])**2.0 + (senPos[m][2])**2.0)**(0.5))

        # Recalculating TDOA by moving starting point to the smallest (or most negative) TDOA value
        mostNeg = min(tdoa)
        tdoa[:] = [x - mostNeg for x in tdoa]

        # Finding sensor with smallest non-zero delay
        tdoa1 = second_smallest(tdoa)
        senNum1 = tdoa.index(tdoa1)

        # Coordinates of sensor with the smallest non-zero delay
        x1 = senPos[senNum1][0]
        y1 = senPos[senNum1][1]
        z1 = senPos[senNum1][2]

        # Constructing MLAT matrix
        A = []
        for m in range(0,4):
            x = senPos[m][0]
            y = senPos[m][1]
            z = senPos[m][2]
            Rm1 = tdoa[m]*vSound
            R21 = tdoa1*vSound
            Am = (x*R21 - x1*Rm1)
            Bm = (y*R21 - y1*Rm1)
            Cm = (z*R21 - z1*Rm1)
            Dm = -((senOrgDist[m]**2.0)*R21 - (senOrgDist[senNum1]**2.0)*Rm1 - Rm1*R21*(Rm1-R21))*0.5
            A += [[Am,Bm,Cm,Dm]]

        # Solving MLAT matrix using SVD
        A = np.array(A)
        (_,_,v) = np.linalg.svd(A)
        # Getting the minimizer
        w = v[3,:]
        w /= w[3]

        # Actual emitter coordinates
        xin = emPos[0]
        yin = emPos[1]

        # Calculated emitter coordinates
        xout = w[0]
        yout = w[1]

        # Calculating the percentage difference between actual and calculated coordinates
        errAtDistX.append(100*abs(xin-xout)/xin)
        errAtDistY.append(100*abs(yin-yout)/yin)

    # Calculating average percentage error for given separation distance
    errXCoord.append(np.mean(errAtDistX))
    errYCoord.append(np.mean(errAtDistY))

# Plotting the graph of percentage error vs distance of separation
plt.figure(1)
plt.title('Distance Between Sensors Study')
plt.xlabel('Separation Distance /m')
plt.ylabel('Percentage Error /%')
plt.plot(distances,errXCoord,'o')
plt.plot(distances,errYCoord,'o')
# Plotting logarithmic trendline
yplot = np.log(errXCoord)
L, A_log = np.polyfit(distances, yplot, 1)
plt.plot(distances,(np.exp(L*distances+A_log)))
plt.show()
