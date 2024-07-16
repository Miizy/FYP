function [tree, path] = rrt_algorithm(environment, sampling_method, sampling_bias)
    if nargin < 3
        sampling_bias = 0;
    end

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

    % Main RRT loop
    for i = 1:max_iter
        % Sample random point
        rand_point = PointSampler.samplingMethod(sampling_method, x_max, y_max, goal, obstacles, i, max_iter, sampling_bias);
        
        % Find nearest node in the tree
        [nearest_node, nearest_idx] = PathingUtility.findNearest(tree, rand_point);
        
        % Steer towards the random point
        new_point = PathingUtility.steer(nearest_node, rand_point, step_size);
        
        % Check if new point is in free space (collision-free)
        if ~PathingUtility.isCollision(nearest_node, new_point, x_max, y_max, obstacles, step_size)
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
            if norm(new_point - goal) < step_size
                goal_reached = true;
                break;
            end
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