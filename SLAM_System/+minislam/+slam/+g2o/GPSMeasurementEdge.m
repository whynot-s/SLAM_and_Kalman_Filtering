classdef GPSMeasurementEdge < g2o.core.BaseUnaryEdge
%This is taken from the course Github.
    methods(Access = public)
        
        function this = GPSMeasurementEdge()
            this = this@g2o.core.BaseUnaryEdge(2); %the measurement is only 2-long (x and y)
        end
        
        function computeError(this)
            x = this.edgeVertices{1}.estimate();
            this.errorZ = x(1:2) - this.z;
        end
        
        function linearizeOplus(this)
            this.J{1} = ... %jacobian only includes x and y measurements, not the bearing.
                [1 0 0;
                0 1 0];
        end        
    end
end