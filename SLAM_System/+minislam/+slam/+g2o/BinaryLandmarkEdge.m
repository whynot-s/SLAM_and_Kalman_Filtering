classdef BinaryLandmarkEdge < g2o.core.BaseBinaryEdge %this is adapted from the GPSMeasurementEdge code on Github

    methods(Access = public)
        
        function this = BinaryLandmarkEdge()
            this = this@g2o.core.BaseBinaryEdge(2); %this is dimension of measurement - 2 long (z)
            %we use binary edge instead of unary on Github because we need
            %to estimate the landmark vertex too!

        end
        
        function computeError(this)
            x = this.edgeVertices{1}.estimate(); %3-long vehicle vector
            landmark_state = this.edgeVertices{2}.estimate(); %initially, this is just the vehicle position, and the edge 

            estimate_difference = landmark_state - x(1:2); %this is the difference between the vehicle and landmark estimate, initially zero!
            
            %need to be 2 values for error, since the measurement is
            %2-long.
            z = this.z; %measurement
            
            this.errorZ(1) = norm(estimate_difference)-z(1); %first measurment gives norm of difference between landmark and vertex

            estimated_b = atan2(landmark_state(2)-x(2),landmark_state(1)-x(1))-x(3); %this will be compared to the measured b
            this.errorZ(2) = g2o.stuff.normalize_theta(estimated_b-z(2)); %second measurement is bearing measurement, difference between estimated and measured

        end
        
        function linearizeOplus(this)
            %Jacobian adapted from the course Github, which in turn was
            %adapted from: https://github.com/petercorke/robotics-toolbox-matlab/blob/master/RangeBearingSensor.m
            
            %values as defined in error function
            x = this.edgeVertices{1}.estimate();
            landmark_state = this.edgeVertices{2}.estimate();
            dx = landmark_state - x(1:2);
            r = norm(dx)+1e-7; %well this is initially 0 if landmark vertex is initialised at vehicle position, therefore that may be messing the jacobian!
            
            this.J{1} = ...
                [-dx(1)/r -dx(2)/r 0;
                dx(2)/r^2 -dx(1)/r^2 -1];
            this.J{2} = - this.J{1}(1:2, 1:2); %square matrix, without the bearing from x

            
            
        end        
    end
end