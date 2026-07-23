# Ultrasound image acquisition executables

Executable files to show the image of linear or convex ultrasound element of the probe.

The solution is build on top of save image api interface and reatins that feature as well. When the program is started it opens the TCP server socket and waits for a client connection. When the client connects server waits for the prompt form the client to send the next ultrasound frame

>[!WARNING]
>Executables need admin privalages to run
Idk why, will try to remove it when i get some time.