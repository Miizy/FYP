function [tree, path] = rrt_star_algorithm(environment, radius, stop_search_on_sol, sampling_method, sampling_bias)
    if nargin < 4
        sampling_bias = 0;
    end

    start = environment.start;
    goal = environment.goal; 
    x_max = environment.x_max;
    y_max = environment.y_max;
    step_size = environment.step_size;
    max_iter = environment.max_iter;
    obstacles = environment.obstacles;

    % Initialize tree with goal node
    tree = goal;
    parents = -1;  
    costs = inf;
    goal_idx = size(tree, 1);
    
    % Add start node to tree
    tree = [tree; start];
    costs = [costs; 0];
    parents = [parents; -1];

    % Main RRT* loop
    for i = 1:max_iter
        rand_point = PointSampler.samplingMethod(sampling_method, x_max, y_max, start, goal, obstacles, i, max_iter, sampling_bias, costs(goal_idx));
        
        % Find nearest node in the tree
        [nearest_node, nearest_idx] = PathingUtility.findNearest(tree(2:end, :), rand_point);
        nearest_idx = nearest_idx + 1;
        
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
            nearby_idxs = PathingUtility.findNearby(tree(2:end, :), new_point, radius);
            for j = 1:length(nearby_idxs)
                near_idx = nearby_idxs(j) + 1;
                near_node = tree(near_idx, :);
                % Check if a path through new_point to near_node is shorter
                new_cost = costs(end) + norm(new_point - near_node);
                if new_cost < costs(near_idx)
                    if ~PathingUtility.isCollision(new_point, near_node, x_max, y_max, obstacles, step_size)
                        parents(near_idx) = size(tree, 1);
                        costs(near_idx) = new_cost;
                        % Update the plot to show the new edge
                        plot([new_point(1), near_node(1)], [new_point(2), near_node(2)], 'Color', [0.7 0.7 0.7], 'LineWidth', 1.5);
                        % Propagate the cost update to the descendants
                        [tree, costs, parents] = propagate_cost_update(tree, costs, parents, near_idx);
                    end
                end
            end
            
            % Check if goal is reached
            if norm(new_point - goal) < step_size
                new_cost_to_goal = costs(end) + norm(new_point - near_node);
                if new_cost_to_goal < costs(goal_idx)
                    parents(goal_idx) = size(tree, 1);
                    costs(goal_idx) = new_cost_to_goal;
                end
                if stop_search_on_sol
                    break;
                end
            end
        end
    end
    
    % Trace path back to start
    path = [];
    if parents(goal_idx) ~= -1
        current_idx = goal_idx;
        % Stops once current_idx is first point added to tree (the start point)
        while current_idx > 0
            % get all items in the tree at row current_idx (get point at current_idx)
            % add that point to front of path array
            path = [tree(current_idx, :); path];
            current_idx = parents(current_idx);
        end
    end
end

function [tree, costs, parents] = propagate_cost_update(tree, costs, parents, node_idx)
    children_idxs = find(parents == node_idx);
    for i = 1:length(children_idxs)
        child_idx = children_idxs(i);
        parent_idx = parents(child_idx);
        costs(child_idx) = costs(parent_idx) + norm(tree(parent_idx, :) - tree(child_idx, :));
        [tree, costs, parents] = propagate_cost_update(tree, costs, parents, child_idx);
    end
end