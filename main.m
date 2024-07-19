clc;
clear;
close all;
    
% Load environment
environment = EnvironmentSetup.select_environment(1);
environment = EnvironmentSetup.environment_setup(environment);

% Start timer
mytester = Tester();
mytester = mytester.start_timer();

% Run RRT algorithm
[tree, path] = rrt_star_algorithm(environment, 5*environment.step_size, false, 1, 0.3);
%[tree, path] = rrt_algorithm(environment, 1);
%[tree, path] = bidirectional_rrt_star_algorithm(environment, 5*environment.step_size, 2, 0.3);
%[tree, path] = improved_rrt_algorithm(environment);

% Stop timer and display everything
time_taken = mytester.stop_timer();
disp(['Time taken: ', num2str(time_taken)]);
mytester.Tree = tree;
mytester.Path = path;
node_count = mytester.number_of_nodes();
disp(['Number of nodes in tree: ', num2str(node_count)]);
path_length = mytester.path_length();
disp(['Total path length: ', num2str(path_length)]);

PathingUtility.plot_final_path(path);

%path = PathSmoother.simplify_path(path, environment);
%PathingUtility.plot_final_path(path);
%PathSmoother.plot_smooth_path(path);