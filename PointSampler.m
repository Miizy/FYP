classdef PointSampler
    methods(Static)
        % Selector for Sampling points
        function point = samplingMethod(method_number, x_max, y_max, goal, method_params)
            switch method_number
                case 1
                    point = PointSampler.randomSampling(x_max, y_max);
                case 2
                    point = PointSampler.goalBiasSampling(x_max, y_max, goal, method_params{:})
                case 3
                    point = PointSampler.adaptiveSampling(x_max, y_max, goal, method_params{:})
                case 4
                    point = PointSampler.gaussianBiasSampling(x_max, y_max, goal, method_params{:})
                case 5
                    point = PointSampler.obstacleBasedSampling(x_max, y_max, goal, method_params{:})
                otherwise
                    point = PointSampler.randomSampling(x_max, y_max);
            end
        end

        %Sample from all space - Free & Obstacles
        function point = randomSampling(x_max, y_max)
            point = [x_max * rand(), y_max * rand()];
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
        function point = adaptiveSampling(x_max, y_max, goal, iteration, maxIterations)
            bias = min(1, iteration / maxIterations);
            if rand <= bias
                point = goal;
            else
                point = [x_max * rand(), y_max * rand()];
            end
        end

        % Samples a point using gaussian distribution centered at goal 
        function point = gaussianBiasSampling(x_max, y_max, goal, stddev)
            x = normrnd(goal(1), stddev);
            y = normrnd(goal(2), stddev);
            
            % Ensure the sampled point is within the bounds [0, x_max] and [0, y_max]
            x = min(max(x, 0), x_max);
            y = min(max(y, 0), y_max);
            
            point = [x, y];
        end
        
        % Samples a point with a bias towards obstacle boundaries.
        function point = obstacleBasedSampling(x_max, y_max, goal, obstacles, bias)
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