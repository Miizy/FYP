clc;
clear;
close all;

% Load environment
environment_setup;

% Run RRT algorithm
[tree, path] = rrt_algorithm(start, goal, x_max, y_max, step_size, max_iter, obstacles);

% Plot the final path
if ~isempty(path)
    plot(path(:,1), path(:,2), 'g', 'LineWidth', 2);
    title('RRT Path');
else
    disp('No path found.');
end