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
%[tree, path] = rrt_algorithm(environment, 1, 0.7);
%[tree, path] = rrt_star_algorithm(environment, 5*environment.step_size, false, 5, 0.9);
%[tree, path] = bidirectional_rrt_star_algorithm(environment, 5*environment.step_size, true, 6, 0.3);%[tree, path] = improved_rrt_algorithm(environment);

[tree, path, runtimes] = rrt_star_algorithm_timed(environment, 5*environment.step_size, false, 1, 0.9);

% Assume `runtimes` is your x-by-1 double array containing recorded times.
num_points = (1:length(runtimes)) * 10;  % Each time corresponds to 10 nodes

% Create the plot
figure;
plot(num_points, runtimes, '-o', 'LineWidth', 2, 'MarkerSize', 1);

% Add labels and title
xlabel('Number of Nodes', 'FontSize', 12);
ylabel('Elapsed Time (seconds)', 'FontSize', 12);
title('RRT* Runtime vs. Number of Nodes Added', 'FontSize', 14);

% Add grid for better readability
grid on;

% Stop timer and display everything
time_taken = mytester.stop_timer();
disp(['Time taken: ', num2str(time_taken)]);
mytester.Tree = tree;
mytester.Path = path;
node_count = mytester.number_of_nodes();
disp(['Number of nodes in tree: ', num2str(node_count)]);
path_length = mytester.path_length();
disp(['Total path length: ', num2str(path_length)]);

%PathingUtility.plot_final_path(path);

%path = PathSmoother.simplify_path(path, environment);
%PathingUtility.plot_final_path(path);
%PathSmoother.plot_smooth_path(path);