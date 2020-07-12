% This class implements an event-based estimation system using g2o and
% the barebones for building up a minimal, ideal SLAM system. The system is
% event-based and responds to a sequence of events which are time stamped
% and served in order. To implement your SLAM system, you will need to
% implement various methods which mostly involve working with the graph.
% These methods are initially stubbed out and will generate exceptions if
% you try to call them.

classdef G2OSLAMSystem < minislam.slam.VehicleSLAMSystem
    
    properties(Access = protected)
                
        % Flag to run the detailed graph validation checks
        validateGraphOnInitialization;        
        
        % The graph used for performing estimation.
        graph;
        
        % The optimization algorithm
        optimizationAlgorithm;
        
        % The most recently created vehicle vertex.
        currentVehicleVertex;
        
        % The set of all vertices associated with the vehicle state over
        % time.
        vehicleVertices;
        vehicleVertexId;
        
        % The landmark vertices. Confusingly enough, "Map" here refers to
        % the data structure which is used to store the landmarks. (It
        % allows random access of landmarkID to landmark object.)
        landmarksMap;
        
        Remove_Odometry_Flag;
        
        Random_Graph_Pruning_Flag;
        
        Sophisticated_Graph_Pruning_Flag;
    end
        
    methods(Access = public)
        
        % Create the localization system and start it up.
        function this = G2OSLAMSystem()
            
            % Call the base class constructor
            this = this@minislam.slam.VehicleSLAMSystem();
            
            % Create the graph and the optimization algorithm
            this.graph = g2o.core.SparseOptimizer();
            algorithm = g2o.core.LevenbergMarquardtOptimizationAlgorithm();
            %algorithm = g2o.core.GaussNewtonOptimizationAlgorithm();
            this.graph.setAlgorithm(algorithm);
            
            % Do detailed checking by default
            this.validateGraphOnInitialization = true;
            
            % Preallocate; this is a lower bound on size
            this.vehicleVertices = cell(1, 10000);
            
            % No vehicle vertices initally set
            this.vehicleVertexId = 0;
            
            % Allocate the landmark map
            this.landmarksMap = containers.Map('KeyType', 'int64', 'ValueType', 'any');
            
            %Do we want to remove odometry (Q4.2) or not.
            this.Remove_Odometry_Flag = 0;
            
            %For Q4.3, not my answer but is mentioned in the report
            this.Random_Graph_Pruning_Flag = 0;
            
            %For Q4.3 - real answer, set this to 1 and the above to 0.
            this.Sophisticated_Graph_Pruning_Flag = 0;
        end
        
        % Get the underlying g2o graph
        function graph = optimizer(this)
            graph = this.graph;
        end
        
        % Set the flag to enable / disable validation. This spots errors in
        % graph construction, but repeated calls can slow things down by
        % 20% or more.
        function setValidateGraph(this, validateGraphOnInitialization)
            this.validateGraphOnInitialization = validateGraphOnInitialization;
        end
        
        function validateGraphOnInitialization = validateGraph(this)
            validateGraphOnInitialization = this.validateGraphOnInitialization;
        end
        
        % Recommend if an optimization is a good idea. Based on an event,
        % some activities (e.g., such as loop closing) can have a very big
        % impact on the estimates. Therefore, this method returns if the
        % localization algorithm thinks optimizing is a good idea. Here we
        % always return false because this gives the fastest results
        % because you just optimize once, right and the very end
        function recommendation = recommendOptimization(this)
            recommendation = false;
            
            % This is how to do it after every 500 steps
            %recommendation = rem(this.stepNumber, 100) == 0;
        end
        
        function Remove_Odometry(this,amount_of_vehicle_edges_to_keep)
    
            %%%%Once all of the edges and vertices have been outlined, this
            %%%%is where they are evaluated
            %%%%the whole graph is now already built!
            Edges = this.graph.edges();
            
            %so for 4.2, we delete ALL vehicle kinematics edges!
            %EDIT: as it says in the CW, one method will not work - the
            %method which does not work is deleting ALL edges (matrix
            %becomes singular!)
           
            vehicle_edge_count = 0; %instantiation
            for i = 1:length(this.graph.edges()) %loop through all edges
                current_edge = Edges(i); %this data type will be the cell!
                within_cell = current_edge{1}; %this goes into the cell to get the edge
                %type_of_edge = class(within_cell); %gets type of edge
%                 disp("edge type")
%                 disp(type_of_edge)
                is_it_vehicle_edge = isa(within_cell,'minislam.slam.g2o.VehicleKinematicsEdge');
                if is_it_vehicle_edge == 1 && this.Remove_Odometry_Flag == 1
                    %disp("within")
                    vehicle_edge_count = vehicle_edge_count + 1; %goes from 0 to 1 here, therefore must be greater than 1 in the if
                    %disp("vehicle edge count")
                    %disp(vehicle_edge_count)
                    if vehicle_edge_count > amount_of_vehicle_edges_to_keep
                        %error("stop")
                        this.graph.removeEdge(within_cell); %set to zero, since we are looping through the array, we can't delete and change its size
                    end
                end 
            end    
        end
        
        
        function [previous_connection_indices, next_connection_indices] = Maintain_Connectivity(this,i) 
            %this is used in conjunction with below, to find the edges which connect the graph
            if i == 1
                previous_vertex = this.vehicleVertices{i}; %won't hit any common landmarks anyway because it doesn't connect to any.
            else
                previous_vertex = this.vehicleVertices{i-1};
            end
            current_vertex = this.vehicleVertices{i};
            next_vertex = this.vehicleVertices{i+1};
            
            %loop through each of these and form three vectors to tell us
            %which landmarks they see
            
            loop = [previous_vertex;current_vertex;next_vertex];
          
             for j = 1:length(loop)
                this_vertex = loop(j);
                edges = this_vertex.edges;
                hit_landmarks = [];
                for ii = 1:length(edges)
                    current_edge_cell = edges(ii);
                    current_edge = current_edge_cell{1};
                    is_it_landmark_edge = isa(current_edge,'minislam.slam.g2o.BinaryLandmarkEdge');
                    if is_it_landmark_edge == 1
                        landmark = current_edge.vertices{2};
                        id = landmark.landmarkId;
                        hit_landmarks = [hit_landmarks id];
                    end
                end
                %now the landmarks which are hit are established
                if j == 1
                    previous_seen = hit_landmarks;
                elseif j == 2
                    current_seen = hit_landmarks;
                elseif j == 3
                    next_seen = hit_landmarks;
                end
             end
            %NOW WE HAVE ALL OF THE HIT LANDMARKS FOR ADJACENT VERTICES
            %let's just return the landmarks which have crossovers
            %REMEMBER to return them as the edge indices of the current
            %vertex, as this is how safe edges are defined!
            
            common_landmarks_with_previous = intersect(previous_seen,current_seen);
            disp("i")
            disp(i)
            disp("length next vert")
            disp(length(next_vertex))
            if length(next_vertex) ~= 0
                common_landmarks_with_next = intersect(current_seen,next_seen);
            end
            %we know the common landmarks, let's find which edges have
            %connections to these landmarks (from the current vertex)
            current_edges = current_vertex.edges;
            previous_connection_indices = [];
            next_connection_indices = [];
            for k = 1:length(current_edges)
                    current_edge_cell = current_edges(k);
                    current_edge = current_edge_cell{1};
                    is_it_landmark_edge = isa(current_edge,'minislam.slam.g2o.BinaryLandmarkEdge');
                    if is_it_landmark_edge == 1
                    landmark = current_edge.vertices{2};
                    id = landmark.landmarkId;
                    common_w_previous = ismember(id,common_landmarks_with_previous); %these should return 1 or 0
                    if length(next_vertex) ~= 0
                    common_w_next = ismember(id,common_landmarks_with_next);
                    end
                    if common_w_previous == 1
                        previous_connection_indices = [previous_connection_indices k]; %This should return the edge indices, based on the current vertex
                    end
                    if length(next_vertex) ~= 0 %must come first, because if it is last one then common_w_next undefined
                    if common_w_next == 1  %split into two if functions because otherwise the previous takes precedence
                        next_connection_indices = [next_connection_indices k];
                    end
                    end
                    end
            end
            
        end
        
        %below function should get rid of all but two edges on all
        %landmarks and vertices
        function Sophisticated_Graph_Pruning(this,edges_to_keep_per_vehicle,edges_to_keep_per_landmark)
            %firstly we loop through all vertices and make sure that they
            %all have at least two edges attached to them (can be random)
            
            
            vertices = this.vehicleVertices; %instantiated as a huge cell
            amount_of_vertices = length(vertices);
            
            for i = 1:amount_of_vertices
                if i == 5666 %Quality/Operations check
                    disp("stop")
                end
                if i == 1
                    %do nothing for first vertex, irrelevant
                else
                current_vertex = vertices{i}; %do curly bracket for cell
                
                if length(current_vertex) == 0
                    break %this is the end of the actual vertices, rest is just empty cell!
                end
                
                
                current_edges = current_vertex.edges; %all of these are binary landmark edges
                amount_of_current_edges = length(current_edges);
                
                %now we should loop through all of the edges on vertex
                safe_edges = [];
                
                for j = 1:amount_of_current_edges
                        
                    %now we need to identify all of the landmarks which
                    %this vertex connects to.
                    %separating into its own loop here!
                    
                    current_edge = current_edges{j};
                    
                    %there is the edge case that there still exists one
                    %vehicle kinematics edge:
%                     disp("i:")
%                     disp(i)
                    is_it_landmark_edge = isa(current_edge,'minislam.slam.g2o.BinaryLandmarkEdge');
                    if is_it_landmark_edge == 1
%                         disp("i:")
%                         disp(i)
%                         disp("j")
%                         disp(j)
                        landmark_connection_vertex = current_edge.vertices{2}; %type: landmark vertex

                        landmark_connection = landmark_connection_vertex.landmarkId;
                        %now we know which landmark this system is connected
                        %to.
                        %now we need to know how many edges this landmark has
                        edges_on_landmark = length(landmark_connection_vertex.edges);
                        if edges_on_landmark <= edges_to_keep_per_landmark
                            safe_edges = [safe_edges j];

                        elseif j >= amount_of_current_edges - edges_to_keep_per_landmark && length(safe_edges) < edges_to_keep_per_vehicle
                            %There is the situation where we get to the last two edges and none are safe, in that case
                            %then the last two edges need to be safe, no matter
                            %what.

                            safe_edges = [safe_edges j]; %might just get second last edge, but that's okay!
                        end
                    end
                end
                %NOW WE MUST EXPAND THE SAFE EDGES TO MAINTAIN
                %CO0NNECTIVITY IN THE GRAPH - CHECK THE PREVIOUS
                %VERTEX LANDMARK OBSERVATIONS & THE NEXT ONES AND
                %MAKE SURE (IMPORTANT!!) THAT THERE IS AT LEAST ONE
                %LANDMARK CROSSOVER BETWEEN THEM!
                [previous_connection_indices,next_connection_indices] = this.Maintain_Connectivity(i); %this returns the indices  
                %the safe edges should feature at least one from
                %each of these vectors!
                if (i ~= 2)  %the first vertex has no landmark edges and so the second vertex wont have any connections to it.
                    if (i ~= 1) %this one may be unecessary as first vertex never would get to here.
                        previous_connection_matrix = ismember(safe_edges,previous_connection_indices);
                        previous_connection = nnz(previous_connection_matrix); %IF THIS IS ZERO THEN THERE IS NO CURRENT CONNECTION
                        if previous_connection == 0
                            %just add in at least one edge to the safe
                            %edges which connects to the previous vertex
                            safe_edges = [safe_edges previous_connection_indices(1)];
                        end
                    end
                end
                
                next_vertex = vertices{i+1};
                if length(next_vertex) ~= 0
                    next_connection_matrix = ismember(safe_edges,next_connection_indices); %just add the first one, more reliable than random
                    next_connection = nnz(next_connection_matrix);   
                    if next_connection == 0
                        safe_edges = [safe_edges next_connection_indices(1)];
                    end
                end
                
                %now we have defined the edges which MUST be saved
                %NOTE THAT THESE ARE JUST DEFINED IN THE ORDER WHICH THEY
                %APPEAR ! ASSUMING THIS IS CONSTANT!
                
                
                %now we continue to loop through the vertices and delete the non-safe
                %edges, making sure that each of them has the minimum
                %amount of edges

                %firstly let's just do it so that every vertex only has 2
                %edges
                if amount_of_current_edges <= edges_to_keep_per_vehicle
                    %just don't delete any edges
                else
                    
                   %now we delete all edges which are not safe
                   for k = 1:amount_of_current_edges
                       current_edge = current_edges{k};
                       is_it_landmark_edge = isa(current_edge,'minislam.slam.g2o.BinaryLandmarkEdge');
                       if is_it_landmark_edge == 1

                           is_it_safe_matrix = ismember(k,safe_edges); %CHECK TO SEE IF THIS WORKS WITH THE CUSTOM TYPES!
                           is_it_safe = nnz(is_it_safe_matrix); 
                           if is_it_safe == 0 %it is not safe
                               this.graph.removeEdge(current_edge);
                           end   
                       end
                   end
                end
                new_edges = current_vertex.edges; %should either be two or length of safe edges!
            end
            
            end
        end
            

        
        
        function Random_Graph_Pruning(this,percentage_edges_to_keep)
            
            %THIS SYSTEM ONLY ENSURES THAT ALL LANDMARKS HAVE AT LEAST TWO
            %EDGES, NOT VEHICLE VERTICES, WHICH COULD HAVE ZERO!
            
            
            amount_of_landmarks = length(this.landmarksMap);
            for j = 1:amount_of_landmarks
                if j == 1000000 %this code can be used to pause the code at a certain vertex, for debugging purposes
                    disp("at the target node")
                end
                edges_of_landmark = this.landmarksMap(j).edges;
                amount_of_landmark_edges = length(edges_of_landmark); %THIS LISTS ALL OF THE EDGES WHICH ARE ATTACHED TO 
                edges_to_keep = round((percentage_edges_to_keep/100)*amount_of_landmark_edges);
                disp("edges to keep")
                disp(edges_to_keep)
                if edges_to_keep < 3
                    disp("trying to get rid of all edges !")
                    %error("trying to get rid of all edges!")
                    edges_to_keep = 2;
                end
                %Confirmed that the above treats the percentage correctly!
                
                edges_to_delete = randsample(edges_of_landmark,amount_of_landmark_edges - edges_to_keep); %edges_to_keep PER LANDMARK!
                amount_of_edges_to_delete = length(edges_to_delete);
                for i = 1:amount_of_edges_to_delete %looping through all landmark edges to delete!
                    current_landmark_edge_cell = edges_of_landmark(i);
                    current_landmark_edge = current_landmark_edge_cell{1};
                    this.graph.removeEdge(current_landmark_edge);
                    
                end
                %confirmed that the above only leaves X edges remaining on
                %each landmark!
            end
        end
        
        
     
  
        
        % This method runs the graph optimiser and the outputs of the
        % optimisation are used as the initial conditions for the next
        % round of the graph. This makes it possible to iterative compute
        % solutions as more information becomes available over time.
        function optimize(this, maximumNumberOfOptimizationSteps)
            if this.Remove_Odometry_Flag == 1
                this.Remove_Odometry(1); 
                %input (1) is the amount of edges to keep
            end
            if this.Random_Graph_Pruning_Flag == 1
                this.Random_Graph_Pruning(10); %input is amount of edges to keep PER LANDMARK.
            end
            if this.Sophisticated_Graph_Pruning_Flag == 1
                this.Sophisticated_Graph_Pruning(10,10)
            end
            %IMPORTANT: in the instantiation of "this", make sure to change
            %"this.Remove_Odometry_Flag" to 1 or 0, depending on if you
            %would like to remove vehicle edges or not.
            
            this.graph.initializeOptimization(this.validateGraphOnInitialization);
            if (nargin > 1)
                this.graph.optimize(maximumNumberOfOptimizationSteps);
            else
                this.graph.optimize();
            end
        end
                
        % Return the current mean and covariance estimate of the robot.
        % This is only valid after optimization has been called.
        function [x, P] = robotEstimate(this)
            
            %disp("within robotEstimate")
            [xS, PS] = this.graph.computeMarginals(this.currentVehicleVertex);
            x=full(xS);
            P=full(PS);
            
        end
        
        function [T, X, P] = robotEstimateHistory(this)
            
            % Extract the graph
            [xS, PS] = this.graph.computeMarginals();
            % Create the output array
            X = zeros(3, this.vehicleVertexId);
            P = zeros(3, this.vehicleVertexId);
            T = zeros(1, this.vehicleVertexId);
            
            % Copy the outputs over
            for v = 1 : this.vehicleVertexId
                
             %DELVING INTO ONE PARTICULAR VERTEX
            if v == 100000 %this code can be used to pause the code at a certain vertex, for debugging purposes
                disp("vehicle vertex")
                disp(v)
                this_vertex = this.vehicleVertices{v};
                edges = this_vertex.edges
                hit_landmarks = [];
                for i = 1:length(edges)
                    current_edge_cell = edges(i);
                    current_edge = current_edge_cell{1};
                    landmark = current_edge.vertices{2};
                    id = landmark.landmarkId;
                    hit_landmarks = [hit_landmarks id];
                end
                disp("landmarks which this system sees")
                disp(hit_landmarks)
            end
            
            
                idx = this.vehicleVertices{v}.hessianIndex();
                
                T(v) = this.vehicleVertices{v}.time();
                
                % Copy the estimate into the array. If the vertices is
                % fixed (conditioned), its estimate is okay. The covariance
                % is not explicitly defined, but has a value of zero.
                % Therefore we fill this manually.
                if (isempty(idx) == true)
                    X(:, v) = this.vehicleVertices{v}.estimate();
                    P(:, v) = zeros(3, 1);
                else
                    X(:, v) = full(xS(idx));
                    P(:, v) = full(diag(PS(idx, idx)));
                    
                    %WE PUT OUR ANALYSIS OF SINGULARITIES HERE - the
                    %vehiclevertexid stays at the max during optimisation,
                    %as this becomes this.graph (sparse optimiser)
                    if abs(P(:,v)) > 100
                        disp("HIGH COVARIANCE DETECTED")
                        keyboard %should pause?
                        disp(P(:,v))
                        disp("Current Vehicle Vertex:")
                        disp(v) %this high covariance is detected on THIS vertex

                        disp("next vehicle vertex")
                        disp(v+1) %this high covariance is detected on THIS vertex

                        disp("previous vehicle vertex")
                        disp(v-1) %this high covariance is detected on THIS vertex

                        % so, we must print out the landmarks which  each vertex
                        % connects to, to check bisection

                        current_vertex = this.vehicleVertices{v};
                        previous_vertex = this.vehicleVertices{v-1};
                        next_vertex = this.vehicleVertices{v+1};
                        
                        previous_previous_vertex = this.vehicleVertices{v-2};
                        next_next_vertex = this.vehicleVertices{v+2};

                        current_vertex_landmarks = [];
                        previous_vertex_landmarks = [];
                        next_vertex_landmarks = [];
                        previous_previous_vertex_landmarks = [];
                        next_next_vertex_landmarks = [];

                        vertices_to_loop = [current_vertex;previous_vertex;next_vertex;previous_previous_vertex;next_next_vertex];

                        for i = 1:length(vertices_to_loop) %go through all three
                            edges = vertices_to_loop(i).edges;
                            amount_of_edges = length(edges);
                            for j = 1:amount_of_edges
                                current_edge_cell = edges(j);
                                current_edge = current_edge_cell{1};
                                is_it_landmark_edge = isa(current_edge,'minislam.slam.g2o.BinaryLandmarkEdge');
                                if is_it_landmark_edge == 1
                                    current_landmark_vertex = current_edge.vertices{2};
                                    current_landmark = current_landmark_vertex.landmarkId;
                                    if i == 1
                                        current_vertex_landmarks = [current_vertex_landmarks current_landmark];
                                    elseif i ==2
                                        previous_vertex_landmarks = [previous_vertex_landmarks current_landmark];
                                    elseif i ==3
                                        next_vertex_landmarks = [next_vertex_landmarks current_landmark];
                                    elseif i == 4
                                        previous_previous_vertex_landmarks = [previous_previous_vertex_landmarks current_landmark];
                                    elseif i == 5
                                        next_next_vertex_landmarks = [next_next_vertex_landmarks current_landmark];
                                    end
                                end

                            end
                        end

                        disp("previous previous vertex landmarks")
                        disp(previous_previous_vertex_landmarks)
                        
                        disp("previous vertex landmarks")
                        disp(previous_vertex_landmarks)

                        disp("current vertex landmarks")
                        disp(current_vertex_landmarks)

                        disp("next vertex landmarks")
                        disp(next_vertex_landmarks)
                        
                        disp("next next vertex landmarks")
                        disp(next_next_vertex_landmarks)
                        
                        keyboard %should pause
                    end
                    
                    
                    
                    
                end
            end
        end
        
        % Return the means and covariances of the landmark estimates. These
        % are only valid after optimization has been called.
        function [x, P, landmarkIds] = landmarkEstimates(this) 
            
            landmarkVertices = values(this.landmarksMap);
            
            numberOfLandmarks = length(landmarkVertices);
            
            landmarkIds = NaN(1, numberOfLandmarks);
            x = NaN(2, numberOfLandmarks);
            P = NaN(2, 2, numberOfLandmarks);
            
            [xS, PS] = this.graph.computeMarginals();
            
            for l = 1 : numberOfLandmarks
                landmarkIds(l) = landmarkVertices{l}.landmarkId(); 
                idx = landmarkVertices{l}.hessianIndex();
                x(:, l) = full(xS(idx));
                if (isempty(idx == true))
                    P(:, :, l) = zeros(3, 3);
                else
                    P(:, :, l) = full(PS(idx, idx));
                end
            end
        end
    end
        
    % These are the methods you will need to overload
    methods(Access = protected)
                        
        % Declare bodies all of these methods. This makes it possible to
        % instantiate this object.
         
        function handleInitialConditionEvent(this, event)
            
            % Add the first vertex and the initial condition edge
            this.currentVehicleVertex = minislam.slam.g2o.VehicleStateVertex(this.currentTime);
            this.currentVehicleVertex.setEstimate(event.data);
            this.currentVehicleVertex.setFixed(true);
            this.graph.addVertex(this.currentVehicleVertex);
            this.vehicleVertexId = 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
       end
       
        function handleNoPrediction(~) %tilde means ignore the input argument
            % Nothing to do
            % if there is no prediction, then there is no vertex which
            % needs to be added 
        end
        
        function handlePredictToTime(this, time, dT)
            %g20 slam system has many properties, including the u input! 
            % Create the next vehicle vertex and add it to the graph
            this.currentVehicleVertex = minislam.slam.g2o.VehicleStateVertex(time);
            this.graph.addVertex(this.currentVehicleVertex);
            % Handle the prediction
            %The first step is to outline the previous datapoint/vertex
            previousX = this.vehicleVertices{this.vehicleVertexId}.estimate; %we do not include -1, since the ID gets updated at the bottom
            %NOTE THAT WE HAVE TO USE .ESTIMATE AS .X IS PRIVATE (OR CONDITIONED ACCESS)!
            
            %Next we have to define the M matrix - the "between pose"
            %model:
            previous_bearing = previousX(3);
            M = [cos(previous_bearing),-sin(previous_bearing),0;
                 sin(previous_bearing),cos(previous_bearing),0;
                 0,0,1];

            u = this.u;
            Q = this.uCov;
            v = randn(3,1); %just zero mean (Unit variance) gaussian noise!
            x_predicted = previousX; %JUST FOR THE SHAPE REALLY
            x_predicted = x_predicted+dT*M*u;
            x_predicted(3) = g2o.stuff.normalize_theta(x_predicted(3));
               
            this.currentVehicleVertex.setEstimate(x_predicted) %the prediction is just a 3x1 matrix - X,Y,Angle 
            
            %now dd the edges
            odometry = this.u;
            %odometry(3) = g2o.stuff.normalize_theta(odometry(3));
            omegaQ = inv(Q);
            process_edge = minislam.slam.g2o.VehicleKinematicsEdge(dT);
            process_edge.setVertex(1,this.vehicleVertices{this.vehicleVertexId}); %previous
            process_edge.setVertex(2,this.currentVehicleVertex); %current
            process_edge.setMeasurement(odometry)
            process_edge.setInformation(omegaQ)
            this.graph.addEdge(process_edge);
            % Bump the indices
            this.vehicleVertexId = this.vehicleVertexId + 1;
            this.vehicleVertices{this.vehicleVertexId} = this.currentVehicleVertex;
        end
        
        function handleGPSObservationEvent(this, event)
            
            % Handle a GPS measurement

            %event.data is the GPS readings
            
            %event.covariance allows for the calculation of covariance
            %matrix
            
            current_vehicle_vertex = this.vehicleVertices{this.vehicleVertexId};
            omegaR = inv(event.covariance); %information matrix for measurements
            
            GPS_edge = minislam.slam.g2o.GPSMeasurementEdge();
            GPS_edge.setVertex(1,current_vehicle_vertex); %only need to add once vertex as it is a unary measurement edge
            GPS_edge.setMeasurement(event.data);
            GPS_edge.setInformation(omegaR);
            this.graph.addEdge(GPS_edge);
        end
        
        function handleLandmarkObservationEvent(this, event)
            
            % Iterate over all the landmark measurements
            for l = 1 : length(event.landmarkIds)
                % Get the landmark vertex associated with this measurement.
                % If necessary, a new landmark vertex is created and added
                % to the graph.
                [landmarkVertex, newVertexCreated] = this.createOrGetLandmark(event.landmarkIds(l));
                z = event.data(:, l); %I believe that this is the measurement from the vehicle to the landmark.

                %so z is the measurement from the platform to THIS
                %individual landmark.
                
                %therefore, to find the absolute position of the landmark,
                %we need the platofrm position first
                %platform_position_and_bearing = this.currentVehicleVertex.estimate();
                if newVertexCreated == 1
                    this.graph.addVertex(landmarkVertex);
                end
                    
                    
                    
                    
                platform_position_and_bearing = this.vehicleVertices{this.vehicleVertexId}.estimate();
    
                
            if newVertexCreated == 1 %if the system is only just created, give it an initial estimate
                    %after this initial estimate, we only add edges which
                    %which eventually lead to a better value
                  
                %landmarkVertex.setEstimate(platform_position_and_bearing(1:2)); %INITIALLY estimate the landmark at the vehicle position
                landmarkVertex.setEstimate([0;0]); %instantiating at vehicle position gets stuck in local minima
            end
                
                %add the binary Edge - must be binary to use vehicle info
                %and estimate landmark position
                landmark_edge = minislam.slam.g2o.BinaryLandmarkEdge();
                landmark_edge.setVertex(1,this.vehicleVertices{this.vehicleVertexId}); %current vehicle position
                landmark_edge.setVertex(2,this.landmarksMap(event.landmarkIds(l)));
                landmark_edge.setMeasurement(z) %REAL measurement!
                
                %since we are now using the REAL measurement (rather than
                %processing it to send to the edge), we can use the real
                %measurement covariance.
                
                R = event.covariance;  %this current landmark observation's covariance
                omegaR = inv(R);          
                landmark_edge.setInformation(omegaR)
                this.graph.addEdge(landmark_edge);               
            end        
        end
   end
    
    methods(Access = protected)
        
        % This method returns a landmark associated with landmarkId. If a
        % landmark exists already, it is returned. If it does not exist, a
        % vertex is created and is added to the graph.
        function [landmarkVertex, newVertexCreated] = createOrGetLandmark(this, landmarkId)
            
            % If the landmark exists already, return it
            if (isKey(this.landmarksMap, landmarkId) == true)
                landmarkVertex = this.landmarksMap(landmarkId);
                newVertexCreated = false;
                return
            end
            %so if the landmark has not been seen/indexed before, make a
            %vertex for it!
            landmarkVertex = minislam.slam.g2o.LandmarkVertex(landmarkId); %instantiate it with its id
            newVertexCreated = true;
            
            %we have created the vertex now, so we must surely  add it to
            %the graph too        
            this.landmarksMap(landmarkId) = landmarkVertex; %this assigns the landmark to the actual map. (count should go up)               
        end       
    end
end
