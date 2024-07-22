classdef PointSampler
    methods(Static)
        % Selector for Sampling points
        function point = samplingMethod(method_number, x_max, y_max, start, goal, obstacle, iteration, max_iter, bias, c_max)
            switch method_number
                case 1
                    point = PointSampler.randomSampling(x_max, y_max);
                case 2
                    point = PointSampler.goalBiasSampling(x_max, y_max, goal, bias);
                case 3
                    point = PointSampler.adaptiveSampling(x_max, y_max, goal, iteration, max_iter);
                case 4
                    point = PointSampler.gaussianBiasSampling(x_max, y_max, goal, bias);
                case 5
                    point = PointSampler.obstacleBasedSampling(x_max, y_max, goal, obstacle, bias);
                case 6
                    point = PointSampler.informedRRTStarSampling(x_max, y_max, start, goal, c_max);
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

        function point = informedRRTStarSampling(x_max, y_max, start, goal, c_max)
            if isfinite(c_max)
                c_min = norm(goal - start);
                x_center = (start + goal) / 2;
                C = rotation_to_world_frame(start, goal);
                
                r1 = c_max / 2;
                r_rest = sqrt(c_max^2 - c_min^2) / 2;
                
                L = diag([r1, repmat(r_rest, 1, numel(start) - 1)]);
                
                x_ball = sample_unit_n_ball(numel(start));
                
                point = C * L * x_ball + x_center;
                
                % Ensure the sampled point is within the space [0, x_max] x [0, y_max]
                point = min(max(point, 0), [x_max, y_max]);
                point = [point(1,1), point(2,2)];
            else
                % Uniform sampling in the space [0, x_max] x [0, y_max]
                point = [rand * x_max, rand * y_max];
            end
        end

    end
end

function C = rotation_to_world_frame(x_start, x_goal)
    % Calculate the rotation matrix that aligns the vector with the world frame
    d = x_goal - x_start;
    d = d / norm(d);
    
    I = eye(length(d));
    v = d(:);
    
    if norm(v - I(:,1)) < 1e-10
        % Special case: d is already aligned with the world frame
        C = I;
    else
        % Gram-Schmidt process
        U = [v'; null(v')'];
        Q = orth(U);
        C = Q';
    end
end

function x_ball = sample_unit_n_ball(n)
    % Sample a point uniformly from the unit n-ball
    x_ball = randn(n, 1);
    x_ball = x_ball / norm(x_ball);
    r = rand^(1/n);
    x_ball = r * x_ball;
end
