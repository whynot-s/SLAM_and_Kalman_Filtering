% This script runs the GPS

% Configure to disable other sensor types
parameters = minislam.event_generators.simulation.Parameters();
parameters.enableGPS = true;
parameters.enableLaser = false;

% This says how much simulation time happens between each GPS measurement.
% Change as per the coursework instructions
parameters.gpsMeasurementPeriod = 1;

% Set up the simulator and the output
simulator = minislam.event_generators.simulation.Simulator(parameters);

% Create and run the different localization systems
kalmanFilterSLAMSystem = minislam.slam.kalman.KalmanFilterSLAMSystem();
g2oSLAMSystem = minislam.slam.g2o.G2OSLAMSystem();
%results = minislam.mainLoop(simulator, {g2oSLAMSystem}); %delete kalman filter part to just work with g2o
results = minislam.mainLoop(simulator, {kalmanFilterSLAMSystem, g2oSLAMSystem});
x1 = results{1}.vehicleStateHistory(1,:);
y1 = results{1}.vehicleStateHistory(2,:);
x2 = results{2}.vehicleStateHistory(1,:);
y2 = results{2}.vehicleStateHistory(2,:);
disp("x1: ")
disp(x1(1:5))
disp("y1: ")
disp(y1(1:5))
hold on
plot(x1,y1)
hold on
plot(x2,y2)
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
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory', 'LineWidth', 2)
hold on
plot(results{2}.vehicleStateHistory'-results{2}.vehicleTrueStateHistory','--', 'LineWidth', 2)