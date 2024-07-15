classdef PathingUtility
    methods(Static)
        %Sample from all space - Free & Obstacles
        function rand_point = randomSampling(x_max, y_max)
            rand_point = [x_max * rand(), y_max * rand()];
        end

        % Sample a point, a fixed bias to use goal
        function point = goalBiasSampling(x_max, y_max, goal, bias)
            if rand <= bias
                point = goal;
            else
                point = [x_max * rand(), y_max * rand()];
            end
        end

        % Sample a point, an adaptive bias to use goal as iteration increase
        function point = adaptiveSampling(xMax, yMax, goal, iteration, maxIterations)
            bias = min(1, iteration / maxIterations);
            if rand <= bias
                point = goal;
            else
                point = [x_max * rand(), y_max * rand()];
            end
        end

        % Samples a point using gaussian distribution centered at goal 
        function point = gaussianBiasSampling(xMax, yMax, goal, stddev)
            x = normrnd(goal(1), stddev);
            y = normrnd(goal(2), stddev);
            
            % Ensure the sampled point is within the bounds [0, xMax] and [0, yMax]
            x = min(max(x, 0), xMax);
            y = min(max(y, 0), yMax);
            
            point = [x, y];
        end
        
        % Samples a point with a bias towards obstacle boundaries.
        function point = obstacleBasedSampling(xMax, yMax, goal, obstacles, bias)
            if rand <= bias
                % Choose a random obstacle
                numObstacles = size(obstacles, 1);
                obstacleIndex = randi(numObstacles);
                obstacle = obstacles(obstacleIndex, :);
                
                % Inflate the obstacle slightly (inflate factor)
                inflateFactor = 0.1;
                inflatedObstacle = [
                    obstacle(1) - inflateFactor, ...
                    obstacle(2) - inflateFactor, ...
                    obstacle(3) + 2 * inflateFactor, ...
                    obstacle(4) + 2 * inflateFactor
                ];
                
                % Sample a point around the boundary of the inflated obstacle
                x = inflatedObstacle(1) + rand * inflatedObstacle(3);
                y = inflatedObstacle(2) + rand * inflatedObstacle(4);
                point = [x, y];
            else
                point = [x_max * rand(), y_max * rand()];
            end
        end

    end
end