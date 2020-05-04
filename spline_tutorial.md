# A very brief 6dof Trajectory spline tutorial by examples

Please refer to Section IV A of [this paper](http://rpg.ifi.uzh.ch/docs/RSS15_Mueggler.pdf) for definition and 
mathematics. To initialize a spline trajectory given chronologically ordered pose samples:
```python
import uzh_fpv.bspline as bspline
traj = bspline.trajectoryFromSamples(sample_times, sample_poses, dt)
```
This will result in a spline whose control nodes are equally spaced by a period of `dt`, starting at `sample_times[0]`.
See [scripts/evaluate_discretizations.py](scripts/evaluate_discretizations.py) for an experiment where different `dts`
are used to express splines that approximate any given UZH FPV sequence.
These are **not** optimal approximations - for that, see the section on `fit_spline_to_trajectory.py` below.

This trajectory can now be sampled at any continuous time:
```python
# Returns None if undefined:
T_W_B = traj.sample(t)
```

This numeric trajectory can be converted to an optimizeable, *symbolic* trajectory as follows:
```python
import uzh_fpv.bspline_opt as bspline_opt
sym_traj = bspline_opt.SymbolicTrajectory(traj, is_var=True)
```
This expresses each control pose as the control pose of the numeric trajectory concatenated with a twist which is a
[Casadi](https://web.casadi.org/) symbol. If `is_var` is set to `False`, the control poses are instead fixed, leaving only the sampling
time as a variable.

This *symbolic* trajectory can now be sampled either at a constant or a variable time:
```python
sym_T_W_B = sym_traj.sample(t)
sym_T_W_B = sym_traj.sample(t, tvar=sym_t)
```
Note that a numeric time must be provided in both cases. It is used to determine the interval whose polynomial is used
to calculate the sample. The interval `t_i` is chosen such that `t_i <= t <= t_(i+1)`. Ideally, any term that depends
on `sym_T_W_B` should be re-expressed as soon as `tvar` violates this condition in order to switch to the appropriate 
interval.

See the following scripts for example optimizations:

### [scripts/fit_spline_to_gt.py](scripts/fit_spline_to_gt.py)

Optimized variables are the control nodes of the spline. The minimized function is the mismatch between spline sample and 
sampled ground truth poses. These poses are sampled at times which are **fixed**:
```python
import uzh_fpv.casadi_pose as casadi_pose

def error(sym_traj, gt_time, gt_pose):
    """ Returns symbols which can be used as error terms in Casadi. """
    sym_pose = sym_traj.sample(gt_time)
    if sym_pose is not None:
        relpose = casadi_pose.mulNumCas(gt_pose.inverse(), sym_pose)
        return rotationAngle(relpose.R), casadi.norm_2(relpose.t)
    else:
        return None, None
```

### [scripts/align_spline_to_leica.py](scripts/align_spline_to_leica.py)

We have a spline `T_G_I_of_t` which expresses the trajectory of frame `I` in the `G` frame.
Unlike in the previous example, the control poses of the spline (i.e. its shape) are now fixed.
We also have position measurements `p_L_Ps` at times `t_L_Ps` which measure the position of point `P` in reference
frame `L`. `P` is rigidly attached to frame `I`.

The optimized variables are:
* Time offset `t_corr` between position measurements and the spline time.
* Frame transformation `T_L_G` between measurement frame `L` and spline frame `G`
* The rigid position of `P` in `I`, `p_I_P`. 

The following code expresses a vector containing the corresponding error terms, expressing the error between
predicted and measured positions of `P` in frame `L`, **subject to the time offset between spline and measurements**.
```python
# Simplified from the original code:
T_G_I_of_t = bspline_opt.SymbolicTrajectory(num_T_G_I, is_var=False)

t_corr = casadi.MX.sym('tc')
T_L_G_twist = casadi.MX.sym('TLG', 6, 1)
T_L_G = casadi_pose.linearizedPose(initial_guess=num_T_L_G, twist=T_L_G_twist)
p_I_P = casadi.MX.sym('pIP', 3, 1)

errvec = casadi.MX.sym('', 0, 0)
for t_L_P, measured_p_L_P in zip(t_L_Ps, p_L_Ps):
    t_c_P = t_corr + t_L_P
    T_G_I = T_G_I_of_t.sample(t_L_P, tvar=t_c_P)
    p_L_P = T_L_G * T_G_I * p_I_P
    errvec = casadi.vertcat(errvec, casadi_pose.squareNorm(measured_p_L_P - p_L_P))
```
