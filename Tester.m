classdef Tester
    properties
        TimerObj
        StartTime
        EndTime
        Tree
        Path
    end
    
    methods
        function obj = Tester()
            % Constructor: initialize the timer and the tree
            obj.TimerObj = timer;
            obj.Tree = [];
            obj.Path = [];
        end
        
        function obj = start_timer(obj)
            % Start the timer
            obj.StartTime = tic;
        end
        
        function elapsed = stop_timer(obj)
            % Stop the timer and return the elapsed time
            obj.EndTime = toc(obj.StartTime);
            elapsed = obj.EndTime;
        end
        
        function count = number_of_nodes(obj)
            % Function to count the number of items in the tree
            count = size(obj.Tree);
            count = count(1)
        end
        
        function total_length = path_length(obj)
            total_length = 0;
            for i = 1:size(obj.Path, 1) - 1
                % Calculate the Euclidean distance between consecutive points
                distance = norm(obj.Path(i, :) - obj.Path(i + 1, :));
                % Add the distance to the total length
                total_length = total_length + distance;
            end
        end
    end
end
