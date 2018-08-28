# Sound-Localisation

The software was created as a part of my final year project. It was about bio-inspired sound localisation in robotics which could potentially be used in elderly care robots or self-driving cars for the systems to locate the sound source and react appropriately.

The program in Sepratation Distance Study folder should be used to design the microphone array. It was assumed that the microphones will be set on the corners of a square so the program plots a graph of percentage error of the localisation versus the length of the sides of this square. When the target accuracy is known, the separation distance between microphones can be read from the graph.

Sound localisation software contains program allowing to localise the sound source based on recorded mp3 sounds. It employs multilateration and GCC-PHAT algorithms. Testing files have been attached in the folder.

KUKA iiwa is a robotic arm on which the project have been done. The programs in this folder require iiwa stack package and ROS to run. It allows the robotic arm to rotate towards the sound source. Four microphones are needed for this program.
