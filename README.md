Guaranteed Sequential Trajectory Optimization (GuSTO)
=====================================================

ECE 602 Winter 2019 Project

Nurken Tuktibayev (20764609)

Introduction
----------------

Bonalli et al. in their recent paper "GuSTO: Guaranteed sequential trajectory optimization via sequential convex programming" (2019) present an algorithm for solving trajectory optimization problems using Sequential Convex Programming (SCP). SCP is well known as an important tool in optimization, which plays a key role in robot motion planning. This project is an implementation of the proposed GuSTO algorithm in Matlab based on 3D Dubin's Car model dynamics, using CVX, a package for specifying and solving convex programs \[3\],\[4\].


GuSTO Algorithm
-------------------

A new general SCP scheme, named GuSTO (Guaranteed Sequential Trajectory Optimization), has been proposed in \[1\] to solve OCP:

The authors in \[1\] provide the proof of the convergence of GuSTO to a stationary point in the sense of the Pontryagin Maximum Principle.

Data Sources and Functions
------------------------------

**Data Initialization.** Given initial and desired goal points and headings as \[x,y,theta\], the maximum number of iterations and initial time guess. 2 different methods for the initial trajectory can be selected: 1 for a straight line and 0 for a middle point between initial and desired goal points.

Algorithm execution parameters were partially extracted from \[1\], the remaining values were obtained from experiments

**Robot Description.** A simple 3D Dubin's Car model has been selected for the simulation, providing opportunity to focus on the new GuSTO algorithm rather than on familiar, but heavy codes for more complex robot models. The function _DubinsCar()_ creates an object of class DubinsCar with the subfunctions _fDynamic(), ADynamic(), BDynamic(), CostTrue(), InitializeModelParams(), UpdateModelParams(), DynamicConstraints()_ and _ObstacleAvoidanceLinearized()_ .

**World Description.** The function _GenerateObstacles()_ creates obstacles as circles between initial and desired goal points with random center and radius, not allowing any overlaps. In order to simulate algorithm with different parameters for the same initial data, generated matrix world can be saved and retrieved from the .mat file.

**Problem Description.** For the given robot and world descriptions, the task is to find the path that moves the robot gradually from the given start point and heading to the desired goal point and heading while never touching any obstacle with the minimum total control cost. We can define total integral cost as sum between control cost and all the penalties in order to get our original OCP as in (1)-(2). The function _GuSTOProblem()_ creates an object of class GuSTOProblem with the subfunctions _InitialTrajectory(), ConvergenceMetric(), TrustRegionRatio(), ConvexIneqSatisfied()_. 

**Collision Avoidance.** The keep-out zone constraints are generally non-convex, which prevents from using them in a convex optimization. To circumvent the presence of these non-convex constraints, a sequential convex optimization approach is used \[3\]. In this project equations 12(a), 12(b) and 12(c) in \[2\] from \[3\] were implemented in the subfunction _ObstacleAvoidanceLinearized()_ of DubinsCar class for calculating the signed distance between robot and obstacles.

Implementation of GuSTO
---------------------------

Details of GuSTO implementation in MATLAB and the demonstration of the output results can be found in the [html/main.html](https://github.com/nurkenkz/Gusto.m/blob/master/html/main.html) file.

Analysis and Conclusions
----------------------------

In this project proposed GuSTO algorithm was implemented in Matlab and various simulations were accomplished utilizing Dubin's Car model dynamics. GuSTO easily manages avoiding obstacles on 2-dimensional space with relatively small number of iterations and there is no doubt it can be well utilized for more complex robot model dynamics. Linearization of non-convex constraints, such as collision avoidance, provides an advantages for GuSTO compared to other highly efficient techniques. The results demonstrate that this method allows to convexify non-convex constraints by linearized SCP and solve the problem using available convex optimization tools, such as CVX. The algorithm converges relatively fast, proving all of the theoretical conlcusion the authors made in \[1\].

The results of this project and the paper \[1\] can not be compared directly due to two main factors:

*   The authors used several different robot model dynamics for the main results. Although they implemented the Shooting Method for accelerating convergence and simulated it on the same Dubin's Car model, it requires warm start feature, which is not available in CVX.
*   The authors implemented collision avoidance for GuSTO using Bullet Physics engine, which is not available for Matlab.

References
--------------

*   \[1\] R. Bonalli, A. Cauligi, A. Bylard, and M. Pavone, “GuSTO: Guaranteed sequential trajectory optimization via sequential convex programming” in 2019 IEEE International Conference on Robotics and Automation (ICRA), IEEE, may 2019
*   \[2\] J. Virgili-llop, C. Zagaris, R. Zappulla II, A. Bradstreet, and M. Romano, “Convex optimization for proximity maneuvering of a spacecraft with a robotic manipulator,” in AIAA/AAS Space Flight Mechanics Meeting, 2017.
*   \[3\] Michael Grant and Stephen Boyd. CVX: Matlab software for disciplined convex programming, version 2.0 beta. [http://cvxr.com/cvx](http://cvxr.com/cvx), September 2013.
*   \[4\] Michael Grant and Stephen Boyd. Graph implementations for nonsmooth convex programs, Recent Advances in Learning and Control (a tribute to M. Vidyasagar), V. Blondel, S. Boyd, and H. Kimura, editors, pages 95-110, Lecture Notes in Control and Information Sciences, Springer, 2008. [http://stanford.edu/~boyd/graph\_dcp.html](http://stanford.edu/~boyd/graph_dcp.html).
