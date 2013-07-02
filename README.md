Tested on OSX 10.8.4.

* OpenNI-Bin-Dev-MacOSX-v1.5.4.0
* PrimeSense (Stable-5.1.0.41)
* Sensor-Bin-MacOSX-v5.1.2.1
* NITE-Bin-MacOSX-v1.5.2.21
* glew

Install glew as this

    brew install glew

Run as follow

    git clone https://github.com/kinjo/UserTracker3D.git
    cd UserTracker3D
    cd NiUserTracker
    make
    (cd ../Bin/x64-Debug&&./Sample-NiUserTracker)

body gestures
-------------------

![Alt text](http://dl.dropboxusercontent.com/u/3694324/Images/clockwise-rotation.png)

Clockwise rotation

![Alt text](http://dl.dropboxusercontent.com/u/3694324/Images/counterclockwise-rotation.png)

Counterclockwise rotation

![Alt text](http://dl.dropboxusercontent.com/u/3694324/Images/scale-up.png)

Scale up

![Alt text](http://dl.dropboxusercontent.com/u/3694324/Images/scale-down.png)

Scale down

License
--------

Follow original license.

