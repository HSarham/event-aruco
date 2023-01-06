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

    cmake .
    make

## Dataset and Usage
You can download testing data from [here](http://sarmadi.me/public_files/event-aruco/).

Download any of the zip files (for example side2side.zip) and extract it in the same folder as the executable.

Please not that in our paper we only used the modded_default_params/side2side.zip and modded_slow_params/side2side.zip sequences for the quantitative results.

After extracting a sequence you can run the detection algorithm by giving the sequence folder path as input argument. For example:

`./process_events side2side`

The command above also writes down the detections in a file called `detected_markers.txt` in the same dataset folder.

If you want to only watch the dataset's sequence you can use the dataset_player for example by:

`./dataset_player side2side`

## Citation
Please cite our paper if you use our code or dataset:

    @article{Sarmadi_2021,
      doi = {10.1109/access.2021.3058423},
      url = {https://doi.org/10.1109%2Faccess.2021.3058423},
      year = 2021,
      publisher = {Institute of Electrical and Electronics Engineers ({IEEE})},
      volume = {9},
      pages = {27813--27826},
      author = {Hamid Sarmadi and Rafael Munoz-Salinas and Miguel A. Olivares-Mendez and Rafael Medina-Carnicer},
      title = {Detection of Binary Square Fiducial Markers Using an Event Camera},
      journal = {{IEEE} Access}
    }

