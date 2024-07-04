function [tree, path] = bidirectional_rrt_star_algorithm(start, goal, x_max, y_max, step_size, max_iter, obstacles, radius)
    % Initialize tree with start node
    treeA = start;
    parentsA = -1;  % Root node has no parent
    treeB = goal;
    parentsB = -1;  % Root node has no parent
    costsA = 0;  % Cost from the start node to the current node
    costsB = 0; 
    goal_reached = false;

    % Main RRT* loop
    for i = 1:max_iter
        % Sample random point
        rand_point = [x_max * rand(), y_max * rand()];
        
        % Find nearest node in the tree
        [nearest_node, nearest_idx] = PathingUtility.findNearest(treeA, rand_point);
        
        % Steer towards the random point
        new_point = PathingUtility.steer(nearest_node, rand_point, step_size);
        
        % Check if new point is in free space (collision-free)
        if ~PathingUtility.isCollision(nearest_node, new_point, x_max, y_max, obstacles, step_size)
            % Add new point to the tree
            treeA = [treeA; new_point];
            % Compute the cost to the new point
            new_cost = costsA(nearest_idx) + norm(new_point - nearest_node);
            % Add parent of new point to the parent array 
            parentsA = [parentsA; nearest_idx];
            % Add cost of new point to the costs array
            costsA = [costsA; new_cost];
            
            % Plot new edge
            plot([nearest_node(1), new_point(1)], [nearest_node(2), new_point(2)], 'b');
            % Draw a blue circle on the new point
            plot(new_point(1), new_point(2), 'bo');
            drawnow;

            % Rewire the tree
            % Find all nodes within the radius
            nearby_idxs = PathingUtility.findNearby(treeA, new_point, radius);
            for j = 1:length(nearby_idxs)
                near_idx = nearby_idxs(j);
                near_node = treeA(near_idx, :);
                % Check if a path through new_point to near_node is shorter
                new_cost = costsA(end) + norm(new_point - near_node);
                if new_cost < costsA(near_idx)
                    if ~PathingUtility.isCollision(new_point, near_node, x_max, y_max, obstacles, step_size)
                        parentsA(near_idx) = size(treeA, 1);
                        costsA(near_idx) = new_cost;
                        % Update the plot to show the new edge
                        plot([new_point(1), near_node(1)], [new_point(2), near_node(2)], 'g'); % Green line for rewired edges
                    end
                end
            end
            
            nearest_point = PathingUtility.findNearest(treeB, new_point);
            [sigma_near, c_point_idx] = connectGraphs(treeB, nearest_point, new_point, step_size, costsB, new_cost, x_max, y_max, obstacles);
            if ~isempty(sigma_near)
                treeA = [treeA; sigma_near];
                new_cost = new_cost + norm(sigma_near - new_point);
                parentsA = [parentsA; numel(parentsA)];
                costsA = [costsA; new_cost];
                % Plot new edge
                plot([new_point(1), sigma_near(1)], [new_point(2), sigma_near(2)], 'b');
                % Draw a blue circle on the new point
                plot(sigma_near(1), sigma_near(2), 'bo');
                goal_reached = true;
                break;
            end
        end
        [treeA, treeB, parentsA, parentsB, costsA, costsB] = swapTree(treeA, treeB, parentsA, parentsB, costsA, costsB);
    end
    
    % Trace path back to start
    path = [];
    tree = [treeA; treeB];
    if goal_reached
        path = [sigma_near];
        % Set current_idx to last point added to tree (the goal point)
        current_idx = size(treeA, 1);
        % Stops once current_idx is first point added to tree (the start point)
        while current_idx > 0
            % get all items in the tree at row current_idx (get point at current_idx)
            % add that point to front of path array
            path = [treeA(current_idx, :); path];
            current_idx = parentsA(current_idx);
        end
        current_idx = c_point_idx;
        while current_idx > 0 
            path = [path; treeB(current_idx, :)];
            current_idx = parentsB(current_idx);
        end
    end
end

function [treeA, treeB, parentsA, parentsB, costsA, costsB] = swapTree(treeA, treeB, parentsA, parentsB, costsA, costsB)
    tempTree = treeB;
    treeB = treeA;
    treeA = tempTree;

    tempParent = parentsB;
    parentsB = parentsA;
    parentsA = tempParent;

    tempCost = costsB;
    costsB = costsA;
    costsA = tempCost;
end

function [sigma_near, x_near_idx] = connectGraphs(tree, nearest_point, new_point, step_size, costs, new_cost, x_max, y_max, obstacles)
    x_new = PathingUtility.steer(new_point, nearest_point, step_size);
    X_near = PathingUtility.findNearby(tree, x_new, step_size*2);
    L_near = {};
    c_best = inf;
    for i = 1:size(X_near)
        x_near_idx = X_near(i);
        x_near = tree(x_near_idx, :); 
        sigma_near = PathingUtility.steer(new_point, x_near, step_size);
        c_near = costs(x_near_idx) + PathingUtility.costToGo(new_point, x_near) + new_cost;
        L_near = [L_near; {c_near, x_near, sigma_near, x_near_idx}];
    end
    if ~isempty(L_near)
        L_near_sorted = sortrows(L_near, 1);
        for i = 1:size(L_near_sorted)
            [c_near, x_near, sigma_near, x_near_idx] = L_near_sorted{i, 1:4};
            if c_near + PathingUtility.costToGo(new_point, x_near) < c_best
                if ~PathingUtility.isCollision(new_point, x_near, x_max, y_max, obstacles, step_size)
                    return;
                end
            end
        end
    end
    sigma_near = [];
    x_near_idx = -1;
end

function path = generatePath()
end