# Hopping robot code

Zhenyang Chen

2021-11-03

## log

1. code structure is based on hw5, some derivation is borrowed from hw4

2. Finish derive_everything.m, derive system dynamic. Variables are shown in this figure:

   <img src="C:\Users\CZY-Yoga\AppData\Roaming\Typora\typora-user-images\image-20211103184547830.png" alt="image-20211103184547830" style="zoom:67%;" />

   3. Modify corresponding parameters.m to generate parameters.

   4. Modify run-simulation.m: skip the optimization part and turn off some plots for now

   5. Add new animate.m: show the animation of the robot

   6. Modify hybrid_simulation.m: finish continuous + discrete dynamic. We can change the input in **control law** function to conduct different test. Currently no control_law is used (we can modify them after the simulation works).

      

   ## bug:

   run **run_simulation.m**, there are numerical issues when calculating matrix A and b in hybrid_simulation. The problem is probably from the bugs in **derive_everything.m**.