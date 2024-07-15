function [tree, path] = rrt_star_algorithm(start, goal, x_max, y_max, step_size, max_iter, obstacles, radius)
    % Initialize tree with start node
    tree = start;
    parents = -1;  % Root node has no parent
    costs = 0;  % Cost from the start node to the current node
    goal_reached = false;

    % Main RRT* loop
    for i = 1:max_iter
        % Sample random point
        rand_point = [x_max * rand(), y_max * rand()];
        
        % Find nearest node in the tree
        [nearest_node, nearest_idx] = PathingUtility.findNearest(tree, rand_point);
        
        % Steer towards the random point
        new_point = PathingUtility.steer(nearest_node, rand_point, step_size);
        
        % Check if new point is in free space (collision-free)
        if ~PathingUtility.isCollision(nearest_node, new_point, x_max, y_max, obstacles, step_size)
            % Add new point to the tree
            tree = [tree; new_point];
            % Compute the cost to the new point
            new_cost = costs(nearest_idx) + norm(new_point - nearest_node);
            % Add parent of new point to the parent array 
            parents = [parents; nearest_idx];
            % Add cost of new point to the costs array
            costs = [costs; new_cost];
            
            % Plot new edge
            plot([nearest_node(1), new_point(1)], [nearest_node(2), new_point(2)], 'b');
            % Draw a blue circle on the new point
            plot(new_point(1), new_point(2), 'bo');
            drawnow;

            % Rewire the tree
            % Find all nodes within the radius
            nearby_idxs = PathingUtility.findNearby(tree, new_point, radius);
            for j = 1:length(nearby_idxs)
                near_idx = nearby_idxs(j);
                near_node = tree(near_idx, :);
                % Check if a path through new_point to near_node is shorter
                new_cost = costs(end) + norm(new_point - near_node);
                if new_cost < costs(near_idx)
                    if ~PathingUtility.isCollision(new_point, near_node, x_max, y_max, obstacles, step_size)
                        parents(near_idx) = size(tree, 1);
                        costs(near_idx) = new_cost;
                        % Update the plot to show the new edge
                        plot([new_point(1), near_node(1)], [new_point(2), near_node(2)], 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5);
                    end
                end
            end
            
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
