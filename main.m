clc;
clear;
close all;

% Load environment
[start, goal, x_max, y_max, step_size, max_iter, obstacles] = environment_setup();

% Run RRT algorithm
[tree, path] = rrt_algorithm(start, goal, x_max, y_max, step_size, max_iter, obstacles);

PathingUtility.plot_final_path(path);
