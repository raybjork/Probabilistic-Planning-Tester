# Probabilistic Planning Tester

This MATLAB project was created to test the effiency of the RRT* family of planners which represent a conceptually simple answer to the problem of robot planning.  Instead of evaluating the robot’s interactions in 3D space we can consider planning in the free configuration space which will be discovered through random sampling.  This project is a test platform wherein several variations of RRT* can be evaluated against each other on maps created by the user.  The algorithms implemented in this repository include:

  - RRT*
  - RRT*-Smart
  - Informed RRT*
  - {a custom variation of RRT*}
  
For each algorithm, various maps can be tested and an animation of the algorithm in action can be viewed as shown below. To test a single algorithm on a single map the function onetest.m can be used

![image](https://github.com/raybjork/Probabilistic-Planning-Tester/blob/master/rrt_smart.png)

For testing an algorithm many times, or testing it across a number of maps the function multitest.m can be used.  It will return useful information about the performance of each of the algorithms as shown in the image below

![image](https://github.com/raybjork/Probabilistic-Planning-Tester/blob/master/results-example.PNG)
  
This project represents independent research I conducted as part of the course "MEAM 520 - Intro to Robotics" at UPenn.  Read the full discussion of the findings of this research in [this paper](https://github.com/raybjork/Probabilistic-Planning-Tester/blob/master/MEAM_520_Final_Project.pdf)
