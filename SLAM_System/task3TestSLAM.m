% This script runs the more extensive SLAM scenario. This is rather large

% Configure to disable other sensor types
parameters = minislam.event_generators.simulation.Parameters();

% By setting true / false you can enable different combinations of sensors
parameters.enableGPS = true;
parameters.enableLaser = true;

% This changes the period (time between each measurement) of the GPS
parameters.gpsMeasurementPeriod = 1;

% Set this to a finite value causes the simulator to end early
parameters.maximumStepNumber = inf;

% Set up the simulator and the output
simulator = minislam.event_generators.simulation.Simulator(parameters, 'task3');

% Create and run the different localization systems
g2oSLAMSystem = minislam.slam.g2o.G2OSLAMSystem();
%g2oSLAMSystem = minislam.slam.g2o.G2OSLAMSystem_ALTERNATIVE();
results = minislam.mainLoop(simulator, g2oSLAMSystem);

% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')

% You will need to add your analysis code here

% Here's how to print out the number of edges and vertices
g2oGraph = g2oSLAMSystem.optimizer();

numVertices = length(g2oGraph.vertices())
numEdges = length(g2oGraph.edges())

%since the data is desired during the coursework description, I should
%write a script to find the average number of observations by a robot vertex and
%of a landmark respectively.

vertices = g2oGraph.vertices();
total_vertex_edges = 0;
total_landmark_edges = 0;
amount_of_vehicle_vertices = 0;
amount_of_landmark_vertices = 0;
for i = 1:length(vertices)
    current_vertex_cell = vertices(i);
    current_vertex = current_vertex_cell{1};
    
    %now check if it is a vehicle vertex
    is_it_vehicle_vertex = isa(current_vertex,'minislam.slam.g2o.VehicleStateVertex');
    is_it_landmark_vertex = isa(current_vertex,'minislam.slam.g2o.LandmarkVertex');
    if is_it_vehicle_vertex == 1
        %vehicle vertices have landmark edges AND process edges.
        %In tutorial, Simon said that we are only interested in landmark
        %observations:
        edges = current_vertex.edges;
        amount_of_vehicle_vertex_edges = 0; %instantiation and reset
        for j = 1:length(edges)
            current_edge_cell = edges(j);
            current_edge = current_edge_cell{1};
            is_it_landmark_edge = isa(current_edge,'minislam.slam.g2o.BinaryLandmarkEdge');
            if is_it_landmark_edge == 1
                amount_of_vehicle_vertex_edges = amount_of_vehicle_vertex_edges + 1;
            end
        end
        %now amount_of_vehicle_vertex_edges is the amount of landmark
        %observations from that vehicle vertex
        total_vertex_edges = total_vertex_edges+amount_of_vehicle_vertex_edges;
        amount_of_vehicle_vertices = amount_of_vehicle_vertices+1;
        
    elseif is_it_landmark_vertex == 1
        amount_of_landmark_vertex_edges = length(current_vertex.edges);
        total_landmark_edges = total_landmark_edges+amount_of_landmark_vertex_edges;
        amount_of_landmark_vertices = amount_of_landmark_vertices+1;
    else
        error("Unidentified Vertex Type")
    end
    
end
average_robot_vertex_observations = total_vertex_edges/amount_of_vehicle_vertices %note that 2 of this will usually be process edges
%(apart from start and end vertices)

average_landmark_observations = total_landmark_edges/amount_of_landmark_vertices

final_amount_of_landmark_vertices = amount_of_landmark_vertices
final_amount_of_vehicle_vertices = amount_of_vehicle_vertices

