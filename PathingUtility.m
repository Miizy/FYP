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
                    edges = [
                        obs_x, obs_y, obs_x + obs_w, obs_y;
                        obs_x, obs_y, obs_x, obs_y + obs_h;
                        obs_x + obs_w, obs_y, obs_x + obs_w, obs_y + obs_h;
                        obs_x, obs_y + obs_h, obs_x + obs_w, obs_y + obs_h
                    ];
                    for j = 1:4
                        [intersect, ix, iy] = lineSegmentIntersect(from, to, edges(j, 1:2), edges(j, 3:4));
                        if intersect
                            collision = true;
                            return;
                        end
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
    
        function nearby_idxs = findNearby(tree, point, radius)
            % Find all nodes in the tree within the given radius of the point
            distances = sqrt(sum((tree - point).^2, 2));
            nearby_idxs = find(distances <= radius);
        end

        function cost_to_go = costToGo(node, goal)
            % Euclidean distance from node to goal
            cost_to_go = norm(node - goal);
        end

        function [obstacle, distance] = nearest_obstacle_in_way(obstacles, start, goal)
            obstacle = [];
            distance = Inf;
        
            for i = 1:size(obstacles, 1)
                
                obs_x = obstacles(i, 1);
                obs_y = obstacles(i, 2);
                obs_w = obstacles(i, 3);
                obs_h = obstacles(i, 4);
        
                % Define the obstacle rectangle edges
                edges = [
                    obs_x, obs_y, obs_x + obs_w, obs_y;
                    obs_x, obs_y, obs_x, obs_y + obs_h;
                    obs_x + obs_w, obs_y, obs_x + obs_w, obs_y + obs_h;
                    obs_x, obs_y + obs_h, obs_x + obs_w, obs_y + obs_h
                ];
        
                % Check each edge for intersection
                for j = 1:4
                    [intersect, ix, iy] = lineSegmentIntersect(start, goal, edges(j, 1:2), edges(j, 3:4));
                    if intersect
                        % Calculate distance from start point to intersection
                        dist = sqrt((ix - start(1))^2 + (iy - start(2))^2);
                        if dist < distance
                            % Update obstacle and distance if this is the closest intersection found
                            obstacle = obstacles(i, :);
                            distance = dist;
                        end
                    end
                end
            end
        end
    end
end

function [intersect, ix, iy] = lineSegmentIntersect(p1, p2, q1, q2)
    % Check if line segments p1p2 and q1q2 intersect
    x1 = p1(1); y1 = p1(2);
    x2 = p2(1); y2 = p2(2);
    x3 = q1(1); y3 = q1(2);
    x4 = q2(1); y4 = q2(2);

    denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
    if denom == 0
        intersect = false;
        ix = NaN;
        iy = NaN;
        return;
    end

    t = ((x1 - x3)*(y3 - y4) - (y1 - y3)*(x3 - x4)) / denom;
    u = -((x1 - x2)*(y1 - y3) - (y1 - y2)*(x1 - x3)) / denom;

    if t >= 0 && t <= 1 && u >= 0 && u <= 1
        intersect = true;
        ix = x1 + t * (x2 - x1);
        iy = y1 + t * (y2 - y1);
    else
        intersect = false;
        ix = NaN;
        iy = NaN;
    end
end
