# Event-ArUco

A library to detect ArUco markers using event cameras

## Reference implementation of the following paper:
H. Sarmadi, R. Muñoz-Salinas, M. A. Olivares-Mendez and R. Medina-Carnicer, "Detection of Binary Square Fiducial Markers Using an Event Camera," in IEEE Access, vol. 9, pp. 27813-27826, 2021, doi: 10.1109/ACCESS.2021.3058423.

## Tested Requirements
- OpenCV 4.6
- [libcaer 3.3.14](https://gitlab.com/inivation/dv/libcaer/-/tree/3.3.14)
- Qt5 (Widgets component)


## Compilation
Similar to normal CMake projects, for example in the code directory:

`cmake .`

`make `

## Dataset and Usage
You can download testing data from [here](http://sarmadi.me/public_files/event-aruco/)

Download any of the zip files (for example side2side.zip) and extract it in the same folder as the executable.

Then you can run the detection algorithm after compiling the project for example by:

`./process_events side2side`

The command above also writes down the detections in a file called `detected_markers.txt` in the same dataset folder.

If you want to only watch the dataset's sequence you can use the dataset_player for example by:

`./dataset_player side2side`

