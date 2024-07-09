clc;
clear;
close all;
    
% Load environment
[start, goal, x_max, y_max, step_size, max_iter, obstacles] = environment_setup();

mytester = Tester();
mytester = mytester.start_timer();

% Run RRT algorithm
[tree, path] = improved_rrt_algorithm(start, goal, x_max, y_max, step_size, max_iter, obstacles);

time_taken = mytester.stop_timer();
disp(['Time taken: ', num2str(time_taken)]);
mytester.Tree = tree;
mytester.Path = path;
node_count = mytester.number_of_nodes();
disp(['Number of nodes in tree: ', num2str(node_count)]);
path_length = mytester.path_length();
disp(['Total path length: ', num2str(path_length)]);

PathingUtility.plot_final_path(path);