function [tree, path] = improved_rrt_algorithm(environment)
    start = environment.start;
    goal = environment.goal; 
    x_max = environment.x_max;
    y_max = environment.y_max;
    step_size = environment.step_size;
    max_iter = environment.max_iter;
    obstacles = environment.obstacles;
    
    % Initialize tree with start node
    tree = start;
    parents = -1;  % Root node has no parent
    goal_reached = false;
    k = 1;
    effective_step_size = step_size;
    collision_flag = false;

    % Main RRT loop
    for i = 1:max_iter
        % Target Bias Sampling
        [~, distance] = PathingUtility.nearest_obstacle_in_way(obstacles, tree(end, :), goal);
        if distance >= effective_step_size && ~collision_flag
            rand_point = goal;
        else
            rand_point = [x_max * rand(), y_max * rand()];
        end
        
        % Find nearest node in the tree
        [nearest_node, nearest_idx] = PathingUtility.findNearest(tree, rand_point);
        
        % Get Effective sampling points and update step size
        N1 = get_effective_sampling_points(nearest_node, obstacles, max_iter, step_size, k);
        effective_step_size = step_size * exp(k * (N1 / max_iter));
        disp(['N1: ', num2str(N1)]);
        disp(['effective_step_size: ', num2str(effective_step_size)]);

        % Target Bias Sampling
        new_point = nearest_node + effective_step_size * ((rand_point - nearest_node) / PathingUtility.costToGo(rand_point, nearest_node))
        
        % Check if new point is in free space (collision-free)
        if ~PathingUtility.isCollision(nearest_node, new_point, x_max, y_max, obstacles, effective_step_size)
            % Add new point to the tree
            tree = [tree; new_point];
            % Add parent of new point to the parent array 
            parents = [parents; nearest_idx];
            
            % Plot new edge
            % Note to self: MATLAB plot([x1,x2], [y1,y2], 'b') plots a line
            % from (x1,y1) to (x2,y2)
            plot([nearest_node(1), new_point(1)], [nearest_node(2), new_point(2)], 'b');
            % Draw a blue circle on the new point
            plot(new_point(1), new_point(2), 'bo');
            drawnow;
            
            % Check if goal is reached
            if norm(new_point - goal) < effective_step_size
                goal_reached = true;
                break;
            end
            collision_flag = false;
        else
            collision_flag = true;
        end
    end
    
    % Trace path back to start
    path = [];
    if goal_reached
        path = [goal];
        % Set current_idx to last point added to tree (the goal point)
        current_idx = size(tree, 1);
        % Stops once current_idx is first point added to tree (the start point)
        while current_idx > 0
            % get all items in the tree at row current_idx (get point at current_idx)
            % add that point to front of path array
            path = [tree(current_idx, :); path];
            current_idx = parents(current_idx);
        end
    end
end

function N1 = get_effective_sampling_points(nearest_node, obstacles, N, step_size, k)
    num_obstacles = size(obstacles, 1);
    N1 = N;
    radius = step_size * exp(k)

    for i = 1:num_obstacles
        obstacle_pos = obstacles(i, 1:2);
        width = obstacles(i, 3);
        height = obstacles(i, 4);

        distance = PathingUtility.costToGo(nearest_node, obstacle_pos)
        if distance - width <= radius && distance - height <= radius
            N1 = N1 - (radius - distance);
            if N1 <= 0
                N1 = 0;
                return;
            end
        end
    end
end