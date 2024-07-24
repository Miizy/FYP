classdef EnvironmentSetup
    methods(Static) 
        function environment = environment_setup(environment)
            start = environment.start;
            goal = environment.goal;
            obstacles = environment.obstacles;
            
            % Define problem space
            x_max = 20;
            y_max = 20;
            step_size = 0.5;
            max_iter = 1500;
        
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
            environment.x_max = x_max;
            environment.y_max = y_max;
            environment.step_size = step_size;
            environment.max_iter = max_iter;

        end

        function environment = select_environment(environment_number)
            % Define obstacles
            % [x, y, width, height]
            switch environment_number
                case 1
                    start = [0, 0];
                    goal = [19, 19];
                    obstacles = [];
                case 2
                    start = [2, 18];
                    goal = [18, 2];
                    obstacles = [1, 16, 3, 1; 4, 16, 1, 3; 16, 1, 1, 3; 16, 4, 3, 1];
                case 3
                    start = [2, 2];
                    goal = [10, 10];
                    obstacles = [0, 4, 18, 0.5; 17.5, 4, 0.5, 14;
                        2, 17.5, 16, 0.5; 2, 8, 0.5, 10;
                        2, 8, 12, 0.5; 13.5, 8, 0.5, 6];
                case 4
                    start = [0, 0];
                    goal = [19, 19];
                    obstacles = [
                        2, 2, 2, 3; 5, 6, 2, 3;  
                        10, 10, 4, 2; 15, 1, 3, 5;  
                        3, 12, 5, 3;  8, 5, 4, 2;   
                        12, 14, 3, 2; 18, 10, 1, 4; 
                        6, 2, 2, 2; 14, 4, 2, 3;];
                case 5
                    start = [1,1];
                    goal = [19,19];
                    obstacles = [0, 0, 0.5, 20; 0, 0, 20, 0.5;
                        0, 19.5, 20, 0.5; 19.5, 0, 0.5, 20;
                        2, 2, 0.5, 5; 2, 2, 5, 0.5;  
                        0, 8.5, 9.5, 0.5;  4.25, 4, 0.5, 9; 
                        9, 0, 0.5, 6; 6.5, 2, 0.5, 4;
                        11.5, 4.5, 0.5, 9.5; 9, 13.5, 3, 0.5;
                        9, 11.5, 0.5, 2.5; 7, 11.5, 2, 0.5;
                        11.5, 2, 2.5, 0.5; 14, 0, 0.5, 2.5;
                        14, 4.5, 6, 0.5; 16, 2, 4, 0.5;
                        12, 7, 5.5, 0.5; 17, 9, 0.5, 8.5;
                        17, 17, 2.5, 0.5; 14, 9, 0.5, 3;
                        14, 13.5, 3, 0.5; 2, 16, 7, 0.5;
                        2, 10.5, 0.5, 5.5; 11, 17, 6, 0.5];
                otherwise
                    start = [0, 0];
                    goal = [19, 19];
                    obstacles = [4.5, 4.5, 1, 1];
            end

            environment.start = start;
            environment.goal = goal;
            environment.obstacles = obstacles;
        end
    end
end


