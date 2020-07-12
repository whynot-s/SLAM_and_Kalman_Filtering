% This script runs the odometry

% Configure to disable other sensor types
parameters = minislam.event_generators.simulation.Parameters();
parameters.enableGPS = false;
parameters.enableLaser = false;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
parameters.perturbWithNoise = false;

% Set up the simulator and the output
simulator = minislam.event_generators.simulation.Simulator(parameters);

% Create and run the different localization systems
kalmanFilterSLAMSystem = minislam.slam.kalman.KalmanFilterSLAMSystem();
g2oSLAMSystem = minislam.slam.g2o.G2OSLAMSystem();
%results = minislam.mainLoop(simulator, {g2oSLAMSystem});
results = minislam.mainLoop(simulator, {kalmanFilterSLAMSystem, g2oSLAMSystem});
x1 = results{1}.vehicleStateHistory(1,:);
y1 = results{1}.vehicleStateHistory(2,:);
x2 = results{2}.vehicleStateHistory(1,:);
y2 = results{2}.vehicleStateHistory(2,:);
%disp("x1: ")
%disp(x1(1:5))
%disp("y1: ")
%disp(y1(1:5))
hold on
plot(x1,y1)
hold on
plot(x2,y2)

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.optimizationTimes)
hold on
plot(results{2}.optimizationTimes,'+')

% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
hold on
plot(results{2}.vehicleCovarianceHistory', '--')

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
hold on
plot(results{2}.vehicleStateHistory'-results{2}.vehicleTrueStateHistory','--')
