classdef PathingUtility
    methods(Static)
        function plot_final_path(path)
            if ~isempty(path)
                plot(path(:,1), path(:,2), 'g', 'LineWidth', 2);
                title('RRT Path');
            else
                disp('No path found.');
            end
        end

        function total_length = calculate_path_length(path)
            total_length = 0;
            for i = 1:size(path, 1) - 1
                % Calculate the Euclidean distance between consecutive points
                distance = norm(path(i, :) - path(i + 1, :));
                % Add the distance to the total length
                total_length = total_length + distance;
            end
        end

        function collision = isCollision(from, to, x_max, y_max, obstacles, step_size)
            % Check if the from and to points are within the boundaries of the defined problem space
            if from(1) < 0 || from(1) > x_max || from(2) < 0 || from(2) > y_max || ...
               to(1) < 0 || to(1) > x_max || to(2) < 0 || to(2) > y_max
                collision = true;
                return;
            end
        
            % Number of steps to check along the path from 'from' to 'to'
            num_steps = ceil(norm(to - from) / step_size);
        
            % Generate points along the line from 'from' to 'to'
            for i = 0:num_steps
                point = from + (i / num_steps) * (to - from);
        
                % Check if the point is inside any of the obstacles
                for j = 1:size(obstacles, 1)
                    obs_x = obstacles(j, 1);
                    obs_y = obstacles(j, 2);
                    obs_w = obstacles(j, 3);
                    obs_h = obstacles(j, 4);
                    if point(1) >= obs_x && point(1) <= obs_x + obs_w && ...
                       point(2) >= obs_y && point(2) <= obs_y + obs_h
                        collision = true;
                        return;
                    end
                end
            end
            collision = false;
        end

        function [nearest_node, nearest_idx] = findNearest(tree, point)
            % tree - point: Difference between point in the tree and point
            % 2,2 - Euclidean norm, 2 dimension
            distances = vecnorm(tree - point, 2, 2);
            % get index of min distance
            [~, nearest_idx] = min(distances);
            % get point with min distance
            nearest_node = tree(nearest_idx, :);
        end
        
        function new_point = steer(from, to, step_size)
            direction = to - from;
            distance = norm(direction);
            direction = direction / distance;
            new_point = from + min(step_size, distance) * direction;
        end
    end
end