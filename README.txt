G2o Core Library used throughout. My main contribution is within the new SLAM system itself - this is
seen in the following directory:

SLAM_System -> +minislam -> +slam

The above contains both the Kalman Filter system and the SLAM system,
as well as the VehicleSLAMSystem.m executable.

All other code is part of the G2o Library and adjusted by Simon Julier, UCL.
Additionally, this code was written in collaboration with Esmeralda Ypsilanti, UCL.


The SLAM code features additional custom edges (binary and unary) as well
as vertices from the graph-based system (for vehicle and landmarks).
Moreover, different systems can be iterated across and comapred, via the flags described
below and in the code.

I have written some more analysis on the system and its results (including graphical comparisons)
on my website, which is given below:

https://rdanks.wixsite.com/raydanks/post/slam-and-kalman-filtering


Read below before running - make sure that all flags are set to 0 before attempting the tasks which are not Part 4.

All flags can be found in the G20SLAMSystem.m file,
in the property constructor section (near the top).


Run setup.m

This system has not been tested with sparseinv as it sometimes caused problems.
Do not use sparseinv.

A description of the different tasks is given below:

Part 1: Use pure Vehicle Odometry and compare the Kalman Filter and SLAM systems.

Part 2: Include GPS for both KALMAN and G2o slam systems.

Part 3: Implement the SLAM system to recognise landmarks and localise itself.

Part 4.1: Add GPS to the new SLAM System.

Part 4.2: Remove Vehicle Odometry from the system.

Part 4.3: Graph Reduction - Graph Pruning is used. Random Graph Pruning
(less effective) and sophisticated graph pruning (more effective) are toggled via flags.

To run:
Part 1: Run Task1TestPrediction.m
Part 2: Run Task2TestGPS.m
Part 3: Run Task2TestSLAM.m (Task3DevelopSLAM.m is a smaller map)

Part 4.1: Run Task4ReviseSLAM.m and set GPS to true

Part 4.2: set this.Remove_Odometry_Flag to 1.

Part 4.3: The random pruning has been left available, to acces this set this.Random_Graph_Pruning_Flag = 1.

The actual optimal result is not the random pruning, it is the sophisticated pruning.
In order to acces the actual result, set this.Sophisticated_Graph_Pruning = 1 and this.Random_Graph_Pruning_Flag = 0.

If the system hits a vertex which has an extremely high covariance (indicating a failed run and a bisected graph), the code will stop
and go into debug mode so that the user can see which vertex is faulty and garner more information. This should not
happen during the tests provided.

Depending on your system specs, MATLAB may sometimes crash during optimisation - if this is unfortunately the case then
force-close MATLAB and rerun the system.