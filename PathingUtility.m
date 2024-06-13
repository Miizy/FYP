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
    end
end