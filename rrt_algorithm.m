function [tree, path] = rrt_algorithm(start, goal, x_max, y_max, step_size, max_iter, obstacles)
    % Initialize tree with start node
    tree = start;
    parents = -1;  % Root node has no parent
    goal_reached = false;

    % Main RRT loop
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
                disp('Goal reached!');
                goal_reached = true;
                break;
            end
        end
    end
    
    % Trace path back to start
    path = [];
    if goal_reached
        path = [goal];
        current_idx = size(tree, 1);
        while current_idx > 0
            path = [tree(current_idx, :); path];
            current_idx = parents(current_idx);
        end
    end
end