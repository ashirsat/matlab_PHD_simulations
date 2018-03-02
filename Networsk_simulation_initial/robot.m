classdef robot
    %ROBOT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties 
        X,
        Y,
        theta,
        force,
        index,
        data,
        A(5,5)
        sum
    end
    
    methods 
        function obj = robot(identity,X,Y,theta,data)
            % ROBOT Construct an instance of this class
            %   Detailed explanation goes here
            if nargin > 0
                if isnumeric(identity) == 1
                    obj.index = identity;
                    obj.X = X;
                    obj.Y = Y;
                    obj.theta = theta;
                    obj.data = data;
                    obj.sum =0;
                else
                    error("Please enter a number");
                end
            end
        end
        
        function self = getdata(data)
            %GETDATA This function assigns data to the robot
            %   This method assigns the data received by the robot when
            %   communicated by other robots. 
            self.data = data;
        end
        
        function odometry = getodometry(identity,X,Y, theta)
            % GETODOMETRY get the odometry data from the robot motion
            %    This method calcuates the current position of the robot by
            %    integrating the velocity at the previous time instant
            odometry.X = X;
            odometry.Y = Y;
            odometry.theta = theta;
        end
        
        
    end
end

