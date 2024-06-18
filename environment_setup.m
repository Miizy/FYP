function [start, goal, x_max, y_max, step_size, max_iter, obstacles] = environment_setup()
    % Define problem space
    x_max = 20;
    y_max = 20;
    start = [0, 0];
    goal = [19, 19];
    step_size = 0.5;
    max_iter = 1000;

    % Define obstacles (example: a 1x1 square centered at (5,5))
    obstacles = [4.5, 4.5, 1, 1]; % [x, y, width, height]

    % Create figure
    figure;
    hold on;
    axis([0 x_max 0 y_max]);
    plot(start(1), start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

    % Plot obstacles
    for i = 1:size(obstacles, 1)
        rectangle('Position', obstacles(i,:), 'FaceColor', [0.5 0.5 0.5]);
    end
end
