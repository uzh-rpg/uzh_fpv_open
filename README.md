# uzh_fpv_open

![if3](http://rpg.ifi.uzh.ch/datasets/uzh-fpv/trajs/indoor_forward_3_snapdragon_with_gt.gif)
![i452](http://rpg.ifi.uzh.ch/datasets/uzh-fpv/trajs/indoor_45_2_snapdragon_with_gt.gif)
![of1](http://rpg.ifi.uzh.ch/datasets/uzh-fpv/trajs/outdoor_forward_1_snapdragon_with_gt.gif)

This is additional code that we release under the GPL-3.0 license to accompany [the UZH FPV dataset](http://rpg.ifi.uzh.ch/uzh-fpv.html).
It has the following main features:

* [Code to read the raw Leica measurements](python/uzh_fpv/leica.py).
* Code to assess the quality of the provided ground truth (see below for instructions):
  * By matching the predicted gyro output to the actual gyro output, as proposed [in this issue by Vladyslav Usenko](https://github.com/uzh-rpg/IROS2019-FPV-VIO-Competition/issues/6).
  * By matching the predicted position of the prism to Leica measurements.
* As a serendipitous byproduct, [an implementation of a time-differentiable continuous trajectory representation](python/uzh_fpv/bspline_opt.py), see the **[quick tutorial](spline_tutorial.md)**.

This repository does **not** contain the code that was used to generate the ground truth.

# Table of Contents

1. [Citing](#citing)
2. [Installation](#installation)
3. [Usage](#usage)
   * [Calculating the Leica error of an existing ground truth](#calculating-the-leica-error-of-an-existing-ground-truth)
   * [Overview of other scripts](#overview-of-other-scripts)
4. [Acknowledgements](#acknowledgements)

# Citing

Please cite the dataset paper when using the code in this repository:
```bibtex
@InProceedings{Delmerico19icra,
 author        = {Jeffrey Delmerico and Titus Cieslewski and Henri Rebecq and
                  Matthias Faessler and Davide Scaramuzza},
 title         = {Are We Ready for Autonomous Drone Racing? The {UZH-FPV} Drone
                  Racing Dataset},
 booktitle     = {{IEEE} Int. Conf. Robot. Autom. ({ICRA})},
 year          = 2019
}
```
To achieve the matching of ground truth to Leica measurements, this repository contains [an implementation of a time-differentiable continuous trajectory representation](python/uzh_fpv/bspline_opt.py) based on the following publication:
```bibtex
@InProceedings{Mueggler15rss,
 author      = {Elias Mueggler and Guillermo Gallego and Davide Scaramuzza},
 title       = {Continuous-Time Trajectory Estimation for Event-based Vision
               Sensors},
 booktitle   = {Robotics: Science and Systems ({RSS})},
 year        = 2015,
 doi         = {10.15607/RSS.2015.XI.036}
}
``` 
It allows to solve optimization problems that include both the trajectory shape and a time offset on trajectory sampling times.
For optimization, we use [Casadi](https://web.casadi.org/).
```bibtex
@Article{Andersson2019,
 author = {Joel A E Andersson and Joris Gillis and Greg Horn
           and James B Rawlings and Moritz Diehl},
 title = {{CasADi} -- {A} software framework for nonlinear optimization
           and optimal control},
 journal = {Mathematical Programming Computation},
 volume = {11},
 number = {1},
 pages = {1--36},
 year = {2019},
 publisher = {Springer},
 doi = {10.1007/s12532-018-0139-4}
}
```

# Installation

Assuming you have [ROS Melodic installed](http://wiki.ros.org/melodic/Installation) on Ubuntu 18.04:
```bash
sudo apt install python-catkin-tools

mkdir -p uzh_fpv_ws/src
cd uzh_fpv_ws
catkin config --init --mkdirs --extend /opt/ros/melodic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release

cd src
git clone git@github.com:catkin/catkin_simple.git
git clone git@github.com:uzh-rpg/uzh_fpv_open.git
cd uzh_fpv_open
pip install -r requirements.txt

catkin build
```

To interact with the dataset, you will need to add the following folders to the root of the repository.
Note that you can use softlinks (`ln -s /actual/location .`):

| Folder | Description |
|-------|--------| 
| raw | Contains raw data: unzipped "Leica" folder from the [public dataset](http://rpg.ifi.uzh.ch/uzh-fpv.html) or the `raw` folder from the internal dataset. |
| output | Contains the bags (not zips) from the public dataset, or the contents of `v2` from the internal dataset. |

# Usage

Flags defined in [python/uzh_fpv/flags.py](python/uzh_fpv/flags.py) are used to specify which sequence you want to work with. For example, to use `indoor_45_2_snapdragon`, you would use flags `--env=i --cam=45 --nr=2 --sens=snap`.
Graphical output can in most scripts be suppressed with `--nogui`.

## Calculating the Leica error of an existing ground truth

Specify which dataset to use with the above flags. See the [spline tutorial](spline_tutorial.md) to better understand
the involved optimizations.

### 1. Create a cached bag which contains only the necessary data:

```bash
python scripts/make_min_imu_gto_bag.py
```

### 2. Fit a spline to the ground truth

In order to make time offset optimization well-behaved, we approximate the ground truth with a spline.
This spline is then used to help estimate the time offset. 
For the final result, the spline is discarded and the ground truth is again used directly.
```bash
# Spline is written into the 'intermediate' folder in project root.
python scripts/fit_spline_to_gt.py
```
Parameters that could be tweaked in this step are `--dt` and `--errs`.
We suggest to simply use the default values (no need to specify).
They control spline resolution (period between control nodes) and the amount of ground truth samples that are used to express the optimized cost function.
Using every single ground truth sample would be unnecessarily costly. 
See [scripts/evaluate_discretizations.py](scripts/evaluate_discretizations.py).

### 3. Align the spline with the Leica measurements

While this will provide a good estimate of disagreement between the ground truth and the Leica measurements, the main
goal of this step is to establish the time offset between ground truth = sensor timestamps and Leica timestamps.
```bash
# Alignement parameters are written into the 'intermediate' folder in project root.
python scripts/align_spline_to_leica.py
```

### 4. Get errors from ground truth alignment

Takes the initial guess from the alignment of the spline to the Leica measurements to align the full ground truth to 
the Leica measurements and plots the Leica errors. 
```bash
python scripts/report_aligned_gt_error.py
```
![xy_traj](examples/xy_traj.png)
![xyzt_traj](examples/xyzt_traj.png)
![leica_err](examples/leica_err.png)

### 5. Summarize the results from all sequences
```bash
python scripts/summarize_gt_error.py
```
The results of this for the currently published ground truth are reported [in the GitHub issues](https://github.com/uzh-rpg/uzh_fpv_open/issues).

## Overview of other scripts

Found in the scripts folder. Can be categorized as follows:

### Preprocessing

`make_min_imu_gto_bag.py`: Create a `min_bag` containing only imu and ground truth, for faster processing in other scripts.

### Dataset inspection

`compare_gyro.py` (as proposed [in this issue by Vladyslav Usenko](https://github.com/uzh-rpg/IROS2019-FPV-VIO-Competition/issues/6)), `plot_leica.py`, `plot_leica_vs_gt.py`

### Dataset publishing

`batch_check_ground_truths.py`, `batch_degroundtruth.py` (create bags and zips without ground truths, for datasets where ground truth is withheld), `print_files_to_publish.py` (list of files to be published, to verify contents of the public directory)

## Acknowledgements

The authors would like to thank Stefan Gächter, Zoltan Török, and Thomas Mörwald of Leica Geosystems for their
support in gathering our data, and Innovation Park Zürich and the Fässler family for providing experimental space.
Additional thanks go to iniVation AG and Prof. Tobi Delbruck for their support and guidance with the mDAVIS sensors, 
and to Francisco Javier Pérez Grau for his help in the evaluation. 

We would also like to thank Vladyslav Usenko from the Computer Vision Group at TU Munich and Patrick Geneva from the
Robot Perception and Navigation Group at the University of Delaware for their feedback on the dataset. Finally, we would like to thank our colleague Zichao Zhang from the Robotics and Perception Group for his help in the competitions that are associated with the datasets, as well as his feedback during the development of this code.

This work was supported by the National Centre of Competence inResearch Robotics (NCCR) through the Swiss National 
Science Foundation,the SNSF-ERC Starting Grant, and the DARPA Fast Lightweight Autonomy Program.
