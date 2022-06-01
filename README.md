# Foreword
This REAMDE is a modified version of Yuta Itoh's README [here](https://github.com/YutaItoh/3D-Eye-Tracker), changed to accomodate for RealSense, IP stream, and Raspi cam.

# 3D-Printable Gaze-Tracking Glass Frames
![GithubPoster](https://github.com/LucasThurnherr/Printable-3D-Gaze-Tracking-Glass-Frames-and-Software/blob/main/Pictures/GithubPoster.png)
All parts of this frame should be printable with a normal 3D printer, such as an Ender3 with a 0.4mm nozzle.
The links are mirrorable, so print them twice if you want two eye cameras.
Print at least 3 pins per eyecam, preferably a few more, in case some break.

![sepparateprints](https://github.com/LucasThurnherr/Printable-3D-Gaze-Tracking-Glass-Frames-and-Software/blob/main/Pictures/SepparatePrint.png)

# Software for gaze-tracking

This software aims to provide an easy-to-compile C++ implementation of a 3D eye-tracking method.

The software is designed for a 3-D printed wearable eye-tracking scnenario where a user wears a headset with an eye camera(s) that is equipped with infrared (IR) illuminations. The camera can capture close-up shots of the user's eyes.



With this software, you can obtain the following information:

* 3D information:
  * Gaze (normal) vector
  * Eyeball center w.r.t the camera
  * Pupil diameter [m]
  * Horizontal and Vertical angle to an object gazed upon
  * Distance to an object gazed upon
* 2D information:
  * Pupil ellipse size [pixel] and angle [deg.]

[Sample video on youtube of base gaze-tracker by Yuta Itoh](https://www.youtube.com/watch?v=EH6UVQZgvJE)
[Sample video on youtube of final gaze-tracker](https://www.youtube.com/watch?v=2dTPCRO4B4E)


## Prerequisite

Currently, the software is compatible with Visual Studio 2019. The test was done on a 64bit Windows 10 machine only.

Most of external libraries (for vc14, x64) are bundled in `./3D-Eye-Tracker-master.rar/external`, yet you still need to 
install the following three libraries:
* [OpenCV 3.4](http://opencv.org/downloads.html) 
  * We recommend to use [***a windows installer***](x64)
* [Boost C++ Libraries ver. >1.62] USE THE NEWEST VERSION (https://sourceforge.net/projects/boost/files/boost-binaries/1.78.0/)
  * We recommend to use [***a windows installer***](https://sourceforge.net/projects/boost/files/boost-binaries/1.78.0/boost_1_78_0-msvc-14.0-64.exe/download) (boost_1_78_0-msvc-14.0-64.exe)
  * In {PATH}\boost_1_78_0\boost\math\quarternion.hpp, at line 421, change "private" to "public" 
* [GStreamer version mingw_x86_64 Developer] (https://gstreamer.freedesktop.org/download/)
  * Follow this tutorial: https://www.youtube.com/watch?v=IBHgh-OcuL0 until 1:20 to check if it is working properly
  * Follow this to implement it in OpenCV's CMake: https://funvision.blogspot.com/2020/03/compile-opencv-4-for-visual-studio-2019.html
* [RealSense SDK 2.0] (https://www.intelrealsense.com/sdk-2/)
  * After installation, build the RealSense SDK for debug and release (x64)

  Make sure OpenCV, Boost, GStreamer, and RealSense SDK 2.0 are all inserted into your Environment variables and PATHS, as seen here:

  <img src="https://github.com/LucasThurnherr/Printable-3D-Gaze-Tracking-Glass-Frames-and-Software/blob/main/Install/PathExample.png" width="300px" />
  <img src="https://github.com/LucasThurnherr/Printable-3D-Gaze-Tracking-Glass-Frames-and-Software/blob/main/Install/GStreamerPath.png" width="400px" />

  To setup your own Visual Studio project, you need to use [CMake](https://cmake.org/). You can find a windows installer [here](https://cmake.org/download/).
 
## How to compile

1. Clone the repository `git@github.com:LucasThurnherr/Printable-3D-Gaze-Tracking-Glass-Frames-and-Software.git` to your local folder (e.g., `/your-local-path/Printable-3D-Gaze-Tracking-Glass-Frames-and-Software`)
2. Open `./CMakeLists.txt` and edit the OpenCV path and the Boost path depending on your environment. For example, the default values are as follows:
   * `set(BOOST_ROOT "C:/SDK/boost_1_78_0")`
   * `set(OpenCV_DIR "C:/SDK/opencv-3.4.16/build")`
3. Start CMake GUI and set a source code path (e.g., `/your-local-path/3D-Eye-Tracker`) and a project build path (e.g., `/your-local-path/3D-Eye-Tracker/build_vc14_x64`)
<img src="https://raw.githubusercontent.com/YutaItoh/3D-Eye-Tracker/master/docs/cmake-1.png" width="600px" />
4. Press `Generate` buttons, then choose a right build environment (Visual Studio 14 2015 Win64)
<img src="https://raw.githubusercontent.com/YutaItoh/3D-Eye-Tracker/master/docs/cmake-2.png" width="300px" />
<img src="https://raw.githubusercontent.com/YutaItoh/3D-Eye-Tracker/master/docs/cmake-3.png" width="600px" />
5. Open `3d_eye_tracker.sln` in the build folder, and compile `main` project (recommend to set it as a startup project)
<img src="https://raw.githubusercontent.com/YutaItoh/3D-Eye-Tracker/master/docs/setup-1.png" width="250px" />

  Copy "realsense2.dll" from {PATH}\librealsense-master\build\Release to the same folder as "main.cpp""

  Make sure your additional dependencies looks similar to "Install\Additional Dependencies.txt"
  Make sure your additional include libraries looks similar to "Install\Additional Include Directories.txt" (May be smaller if you have opencv_world3416d.lib installed instead)

### Post-Compilation
If the gaze-tracker works with a webcam, it is ready to be converted to the extended gaze tracker.
Replace "3D-Eye-Tracker-master\main\main.cpp" with "main.cpp" in the main repo.
RealSense camera and IP stream must be connected to the program for this version to run.

## How to run
    
### For IP Stream
Set the IP of your camera stream at line 613
Make sure the bool streamcam at line 58 is true

### For Webcams
The default setting uses a sinlge camera via the DirectShow filter. Check your camera name on a video capture program (e.g., Skype) and set the name in `main.cpp`:
``` c++
		case InputMode::CAMERA_MONO:
			eyecams[0]=std::make_unique<eyecamera::EyeCameraDS>("Pupil Cam1 ID0");

```
Comment out line 613, lines 628-632, and line 656
Comment in line 657
Change bool streamcam at line 58 to false
In this case, the program tries to find a camera named `"Pupil Cam1 ID0"`.

### States
#### Building Eye Model
Once the program is started, the eyecam is shown, so you can center your eye in the window
Press 'g' when satisfied
Once started, it initializes a 3D eye model from 2D pupil observatios:

<img src="https://raw.githubusercontent.com/YutaItoh/3D-Eye-Tracker/master/docs/runtime-2.png" width="600px" />

In this initialization step, a user needs to smoothly rotate his/her eye to capture various 2D shapes of the eye.

After the step, we get 3D eye tracking:

<img src="https://raw.githubusercontent.com/YutaItoh/3D-Eye-Tracker/master/docs/runtime-3.png" width="200px" />

#### Calibration
When the eye model is built, a window showing the RealSense cam will appear
There is a green square on the screen, look at the real-world equivalent of that point
press 'h' to take a new RealSense snapshot, to ensure your head is steady
When steady, press 'g' to move to the next state
Take a new snapshot with 'h' to see in the square has moved

The next 8 states have green squares at different points
Look at the real world equivalents of those points, as before
press 'h' to take a new RealSense snapshot, to ensure your head is steady
When steady, press 'g' to move to the next state
Take a new snapshot with 'h' to see in the square has moved

#### Running
Once in calibState == 11, the calibration is done
press 'h' to take a RealSense snapshot
The RealSense window will display object detections squares, the user's gaze with a green square, and the closest object with its centerpoint in red
The program's terminal will display the horizontal and vertical angles and the distance to the centerpoint of the object highlighted in red

### Tips:

Some debug keys are pre-assigned for a better control of the software:
* `p`: Takes some more 2D pupil observations. Useful when estimated 3D eye model is incorrect due to not-well-distributed 2D observations
* `r`: Resets the 3D eye model and 2D observations and restarts the initialization step
* `ESC`: Exit the program 	
* 'h': Takes Realsense Snapshots
* 'g': Moves the program to next calibration state
* 'f': Resets the calibration, but keeps the eye model

# Acknowledgements

This program integrated/modified several existing codes. Especially, 
* [3D-Eye-Tracker](https://github.com/YutaItoh/3D-Eye-Tracker.git) by [Yuta Itoh](https://github.com/YutaItoh)
* [2D pupil detection code](https://github.com/YutaItoh/3D-Eye-Tracker/blob/master/main/pupilFitter.h) by [Jason Orlosky](http://www.jeoresearch.com/research)
* [3D eye model optimization code](https://github.com/LeszekSwirski/singleeyefitter) (./singleeyefitter) by Leszek Swirski 
* [Depth extractor for RealSense](https://github.com/IntelRealSense/librealsense/issues/6239) by user BarRae
