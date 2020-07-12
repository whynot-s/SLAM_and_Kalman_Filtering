classdef KalmanFilterSLAMSystem < minislam.slam.VehicleSLAMSystem
    
    % This is a very minimal Kalman filter; its purpose is to let me do
    % some debugging and comparison. It is not fully featured.
    
    properties(Access = protected)
        
        % Kalman filter mean
        xEst;
        PEst;
        
        % Kalman filter covariance
        xPred;
        PPred;
        
        % Store of the mean and covariance values for the vehicle
        timeStore;
        xEstStore;
        PEstStore;
    end
    
    methods(Access = public)
        
        function this = KalmanFilterSLAMSystem()
            this = this@minislam.slam.VehicleSLAMSystem();
            this.xEstStore = NaN(3, 1);
            this.PEstStore = NaN(3, 1);
            this.xEst = NaN(3, 1);
            this.PEst = NaN(3, 3);
        end
        
        function [x,P] = robotEstimate(this)
            x = this.xEst(1:3);
            P = this.PEst(1:3, 1:3);
        end
        
        function [T, X, PX] = robotEstimateHistory(this)
            T = this.timeStore;
            X = this.xEstStore;
            PX = this.PEstStore;
        end
        
        function [x, P, landmarkIds] = landmarkEstimates(this)
            landmarkIds = [];
            x = NaN(2, 0);
            P = NaN(2, 2, 0);
        end
        
        function recommendation = recommendOptimization(this)
            recommendation = true;
        end
        
        function processEvents(this, events)
            % Handle the events
            processEvents@minislam.slam.VehicleSLAMSystem(this, events);
            
            % Store the estimate for the future
            this.timeStore(:, this.stepNumber) = this.currentTime;
            this.xEstStore(:, this.stepNumber) = this.xEst(1:3);
            this.PEstStore(:, this.stepNumber) = diag(this.PEst(1:3, 1:3));
        end
        
        
        function optimize(~, ~)
            % Nothing to do
        end
    end
       
    methods(Access = protected)
                    
        function handleInitialConditionEvent(this, event)
            this.xEst = event.data;
            this.PEst = event.covariance;
            this.xPred = this.xEst;
            this.PPred = this.PEst;
        end
        
        function handleNoPrediction(this)
            this.xPred = this.xEst;
            this.PPred = this.PEst;
        end
        
        function handlePredictToTime(this, time, dT)
            % You will need to write the code to implement the process
            % model which:
            % 1. Computes this.xPred
            % 2. Computes the Jacobian
            % 3. Computes the process noise
            
            %%%%%%%%%%%%%%%%%%%FIX - NOT USING TIME ??
            
            % Implementing process model:
            %WE NEED TO PAD OUT THIS MATRIX AS X IS 3-LONG AND SO IS U !!
            F = [1,dT,0;
                 0, 1,0;
                 0,0,1];
             
            %The below, G, is the Jacobian.
            %PADDING SO THAT IT WORKS WITH U!
%             G = [(dT^2)/2,0,0;
%                   dT,1,0;
%                   0,0,1]; %%%%%%%%%%%%COULD BE PADDING THIS ONE WRONG!
            
            G = [(dT^2)/2,0,0;
                 0,dT,0;
                 0,0,1];
            u = this.u;

            x_previous = this.xEst; %correct ??
            
            %The only part left is to compute the process noise!
            Q = this.PEst; %OR do i need to define it from the equation? perhaps not because no given sigma.
%             Q = [(dT^3)/3,(dT^2)/2,0;
%                  (dT^2)/2,dT,0;
%                   0,0,1];
            %gain the noise from a multivariate random distribution, with
            %mean 0 and covariance Q
            
%             disp("Q: ")
%             disp(Q)
            
            v = mvnrnd([0,0,0],Q,3); %3 values - one noise value for each state part
            
            
            this.xPred = F*x_previous+G*u+v;
        end
        
        function handleGPSObservationEvent(this, event)
            
            % You will need to write the code to implement the fusing the
            % platform position estimate with a GPS measurement
            
            time = event.time;
            data = event.data;
            disp("data")
            disp(data)
            z = data;
            
            %disp("event")
            %disp(event)
            
            %event: initial_condition
            %vehicle_odometry
            %gps
            %landmark
            %time
            %type
            %data
            %covariance
            R = event.covariance;
            %disp(time)
            
            % KalmanFilterSLAMSystem
            disp("thisGPS: ")
            disp(this)
            
            % 1x3 matrix
            %disp("XEst")
            %disp(this.xEst)

            % 3x3 matrix
            %disp(this.PEst)
            % 3x3 matrix
            %disp(this.PPred)
            P = this.PPred;
            
            Jh = [1,0,0;
                0,1,0];
            
            %start_time = 0;
            %dT = event.time - start_time;
            %this.xEst = event.data;
            %this.PEst = event.covariance;
            
            % You will need to write a Kalman filter update
            %1x3 matrix
            disp("x")
            disp(this.xPred)
            disp("z")
            disp(z)
            x = this.xPred;
            S_k = Jh * P * Jh' + R;
            W_k = P * Jh' * inv(S_k);
            disp("W")
            disp(W_k)
 
            x = x + W_k * (z - x(1:2));
            
            len_state = length(x);
            len_state = eye(len_state);
            P = (len_state - W_k * Jh) * P;
            
            this.xEst = x; %update x and P
            this.PEst = P;

            %error('handleGPSObservationEvent: implement');
        end
        
        function handleLandmarkObservationEvent(this, event)
            error('Not supported; this should only be used to implement odometry and GPS')
        end
 
    end
end