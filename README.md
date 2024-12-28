<h1 align="center">
  UR5 Cascade APF MPC
</h1>
<p align="center">

## Simulation Environment Requirements
To run the simulation, ensure that you have install visual studio.

Install acado MATLAB interface, you can refer
https://acado.github.io/matlab_overview.html


## Compile the Conrtoller
```
mex_create_high_apf.m
mex_create_low_nmpc.m
```
## Run the controller
```
exec_cascade_apf_nmpc.m
```
## Visualize the results
```
plot_error_xyz.m
plot_jointv.m
plot_min_distance.m
plot_robot_movie.m
```
## Related work
The entire project is written very chaotically; it was done in the early stages and contains some errors in quaternion handling. If it's for learning purposes, you can refer to this project. Although it lacks cascade control, the overall structure will be much cleaner.

https://github.com/zhutianqi2006/ur5_simulink_obstacle_mpc

## BibTeX

If you find this code useful for your research, please consider citing:

```bibtex
@ARTICLE{2024_apf_mpc,
      title={Real-Time Dynamic Obstacle Avoidance for Robot Manipulators Based on Cascaded Nonlinear MPC With Artificial Potential Field}, 
      author={Zhu, Tianqi and Mao, Jianliang and Han, Linyan and Zhang, Chuanlin and Yang, Jun},
      journal={IEEE Transactions on Industrial Electronics}, 
      year={2024},
      volume={71},
      number={7},
      pages={7424-7434},
}
```
