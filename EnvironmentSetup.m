classdef EnvironmentSetup
    methods(Static) 
        function [x_max, y_max, step_size, max_iter] = environment_setup(start, goal, obstacles)
            % Define problem space
            x_max = 20;
            y_max = 20;
            step_size = 0.5;
            max_iter = 5000;
        
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
        function [start, goal, obstacles] = select_environment(environment_number)
            % Define obstacles
            % [x, y, width, height]
            switch environment_number
                case 1
                    start = [0, 0];
                    goal = [19, 19];
                    obstacles = [4.5, 4.5, 1, 1];
                case 2
                    start = [2, 18];
                    goal = [18, 2];
                    obstacles = [1, 16, 3, 1; 4, 16, 1, 3; 16, 1, 1, 3; 16, 4, 3, 1];
                otherwise
                    start = [0, 0];
                    goal = [19, 19];
                    obstacles = [4.5, 4.5, 1, 1];
            end
        end
    end
end


