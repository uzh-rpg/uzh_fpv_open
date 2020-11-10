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
