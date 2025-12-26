clear;
clc;
close all;
%% 1. MAZE AND ROBOT CONFIGURATION
% =========================================================================
fprintf('Initializing maze and robot...\n');
% --- Define the "True" Maze ---
maze_size = 6;
start_cell = [3, 4]; % [row, col]
end_cell = [6, 6];   % [row, col]
% Wall Matrices: 1 = wall, 0 = no wall
H_walls_true = zeros(maze_size + 1, maze_size);
V_walls_true = zeros(maze_size, maze_size + 1);
% Add Boundary Walls (DO NOT CHANGE THESE)
H_walls_true(1, :) = 1;
H_walls_true(end, :) = 1;
V_walls_true(:, 1) = 1;
V_walls_true(:, end) = 1;
% --- Custom Internal Walls (Adjusted from user's input) ---
% H_walls_true(5, 3:5) = 1;
% H_walls_true(6, 2) = 1;
% H_walls_true(4, 1) = 1;
% V_walls_true(1:2, 3) = 1;
% V_walls_true(4:6, 5) = 1;
% V_walls_true(3, 6) = 1;
% V_walls_true(6, 2) = 1;
% V_walls_true(5, 3) = 1;

H_walls_true(3, 1:4) = 1;
H_walls_true(5, 3:6) = 1;
V_walls_true(1:2, 5) = 1;
V_walls_true(4:5, 2) = 1;
V_walls_true(6, 4) = 1;
% --- End Configuration 3 ---
% --- Initialize Robot's State and Knowledge ---
robot_cell_pos = start_cell; % Discrete position [row, col]
% The robot only knows the outer walls initially. Its internal map will be updated.
robot_H_walls = zeros(size(H_walls_true));
robot_V_walls = zeros(size(V_walls_true));
robot_H_walls(1, :) = 1; robot_H_walls(end, :) = 1;
robot_V_walls(:, 1) = 1; robot_V_walls(:, end) = 1;

% Variables to track path statistics
exploration_steps = 0;
wall_handles = []; % Handle array to store the dynamically drawn known walls

%% 2. VISUALIZATION SETUP
% =========================================================================
% NOTE: setup_visualization now only draws the arena border, not the internal walls.
[fig, robot_handle] = setup_visualization(maze_size, start_cell, end_cell);
title('Phase 1: Exploration (Black Walls are Discovered Walls, Blue Robot)');
drawnow;
pause(1);

%% 3. PHASE 1: EXPLORATION LOOP (Builds the map dynamically)
% =========================================================================
fprintf('PHASE 1: Starting exploration...\n');
while any(robot_cell_pos ~= end_cell)
    exploration_steps = exploration_steps + 1;
    fprintf('Exploration Step %d: Robot at cell [%d, %d]\n', exploration_steps, robot_cell_pos(1), robot_cell_pos(2));
    
    % STEP 1: SENSE - Update robot's knowledge with walls of the current cell
    % The robot senses the walls of the cell it has just entered.
    [robot_H_walls, robot_V_walls] = sense_walls(robot_cell_pos, H_walls_true, V_walls_true, robot_H_walls, robot_V_walls);
    
    % VISUALIZATION UPDATE: Clear old known walls and draw new set
    delete(wall_handles);
    [wall_handles] = draw_known_walls(maze_size, robot_H_walls, robot_V_walls);
    
    % CRITICAL CHANGE: Force redraw and pause to visually show the sensed walls 
    % before the movement animation begins.
    drawnow;
    pause(1); 
    
    % STEP 2: PLAN - Run Flood Fill on the robot's CURRENT (partial) map
    distance_grid = calculate_flood_fill(maze_size, end_cell, robot_H_walls, robot_V_walls);
    
    % STEP 3: DECIDE - Find the best adjacent cell to move to (greedy downhill move)
    next_cell = find_next_move(robot_cell_pos, distance_grid, robot_H_walls, robot_V_walls);
    
    if isempty(next_cell)
        fprintf('Error: Robot is trapped! No path found based on current knowledge.\n');
        break;
    end
    
    % STEP 4: ACT - Animate the continuous movement to the next cell
    animate_robot_movement(robot_cell_pos, next_cell, robot_handle, maze_size);
    
    % Update robot's discrete position for the next loop
    robot_cell_pos = next_cell;
    
    % No need for an extra pause here, as the sensing pause handles the timing
end

% --- Phase 1 Status Check ---
if all(robot_cell_pos == end_cell)
    fprintf('\nPHASE 1 COMPLETE: Goal reached in %d steps. Map acquisition successful.\n', exploration_steps);
    title('Exploration Complete!', 'FontSize', 14, 'Color', [0.8 0.5 0.2]);
    pause(2);
else
    title('Failed to Reach Goal', 'FontSize', 16, 'Color', 'r');
    return; % Stop if goal was not reached
end

% %% 4. PHASE 2: OPTIMAL PATH RUN
% % =========================================================================
% fprintf('\nPHASE 2: Starting optimal path run...\n');
% 
% % Reset the robot's discrete position to the start
% robot_cell_pos = start_cell;
% % Animate robot moving back to the start position
% animate_robot_movement(end_cell, start_cell, robot_handle, maze_size, true); 
% 
% optimal_steps = 0;
% 
% while any(robot_cell_pos ~= end_cell)
%     optimal_steps = optimal_steps + 1;
%     fprintf('Optimal Step %d: Robot at cell [%d, %d]\n', optimal_steps, robot_cell_pos(1), robot_cell_pos(2));
%     title(sprintf('Phase 2: Optimal Path Run (Step %d)', optimal_steps), 'FontSize', 14, 'Color', 'b');
% 
%     % Step 1 & 2: PLAN - Run Flood Fill on the robot's fully discovered map
%     distance_grid = calculate_flood_fill(maze_size, end_cell, robot_H_walls, robot_V_walls);
% 
%     % Step 3: DECIDE - Find the best adjacent cell to move to
%     next_cell = find_next_move(robot_cell_pos, distance_grid, robot_H_walls, robot_V_walls);
% 
%     if isempty(next_cell)
%         fprintf('Critical Error: Optimal path calculation failed.\n');
%         break;
%     end
% 
%     % Step 4: ACT - Animate the continuous movement
%     animate_robot_movement(robot_cell_pos, next_cell, robot_handle, maze_size);
% 
%     % Update robot's discrete position
%     robot_cell_pos = next_cell;
% 
%     pause(0.05);
% end
% 
% % --- Final Status Message ---
% if all(robot_cell_pos == end_cell)
%     title(sprintf('Goal Reached! Optimal Path: %d steps. (Exploration: %d)', optimal_steps, exploration_steps), 'FontSize', 16, 'Color', 'g');
%     fprintf('\nSuccess! Optimal path executed in %d steps.\n', optimal_steps);
%     fprintf('Exploration Steps: %d, Optimal Steps: %d\n', exploration_steps, optimal_steps);
% else
%     title('Failed to Execute Optimal Path', 'FontSize', 16, 'Color', 'r');
% end
% 

%% ----------------------------------------------------------------------- %
%                      --- HELPER FUNCTIONS ---                            %
% ------------------------------------------------------------------------ %

function [fig, robot_handle] = setup_visualization(maze_size, start_cell, end_cell)
    % Sets up the visualization window, only drawing the outer boundary walls 
    % and the start/end markers. Internal walls will be drawn dynamically.
    fig = figure('Name', 'Maze Solver Simulation', 'NumberTitle', 'off', 'WindowState', 'maximized');
    hold on;
    axis equal;
    axis([0 maze_size 0 maze_size]);
    set(gca, 'XTick', [], 'YTick', [], 'Box', 'on', 'LineWidth', 1.5);
    
    % Draw Arena Border (these are always known)
    plot([0, maze_size], [maze_size, maze_size], 'k-', 'LineWidth', 3); % North
    plot([0, maze_size], [0, 0], 'k-', 'LineWidth', 3); % South
    plot([0, 0], [0, maze_size], 'k-', 'LineWidth', 3); % West
    plot([maze_size, maze_size], [0, maze_size], 'k-', 'LineWidth', 3); % East

    % Mark Start and End points
    start_center = [start_cell(2)-0.5, maze_size-start_cell(1)+0.5];
    end_center = [end_cell(2)-0.5, maze_size-end_cell(1)+0.5];
    text(start_center(1), start_center(2), 'S', 'FontSize', 16, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', [0.2 0.8 0.2]);
    text(end_center(1), end_center(2), 'E', 'FontSize', 16, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', [0.2 0.2 0.8]);
    
    % Create robot object (a blue triangle)
    robot_start_pos = [start_cell(2)-0.5, maze_size-start_cell(1)+0.5];
    % Define triangle shape (points relative to robot's center [0,0])
    robot_points = [-0.2, 0.3, -0.2; -0.25, 0, 0.25]; 
    robot_handle = patch('XData', robot_points(1,:) + robot_start_pos(1), ...
                         'YData', robot_points(2,:) + robot_start_pos(2), ...
                         'FaceColor', 'b', 'EdgeColor', 'k', 'LineWidth', 1);
    hold off;
end
% -------------------------------------------------------------------------
function wall_handles = draw_known_walls(maze_size, H_walls, V_walls)
    % Draws only the internal walls known to the robot (not boundaries).
    % Returns handles to the plotted wall segments for later clearing.
    wall_handles = [];
    hold on;
    % Draw Known Horizontal Walls (excluding boundaries)
    for r = 2:size(H_walls, 1)-1
        for c = 1:size(H_walls, 2)
            if H_walls(r, c) == 1
                h = plot([c-1, c], [maze_size-r+1, maze_size-r+1], 'k-', 'LineWidth', 2.5);
                wall_handles(end+1) = h;
            end
        end
    end
    % Draw Known Vertical Walls (excluding boundaries)
    for r = 1:size(V_walls, 1)
        for c = 2:size(V_walls, 2)-1
            if V_walls(r, c) == 1
                h = plot([c-1, c-1], [maze_size-r, maze_size-r+1], 'k-', 'LineWidth', 2.5);
                wall_handles(end+1) = h;
            end
        end
    end
    hold off;
end
% -------------------------------------------------------------------------
function [robot_H, robot_V] = sense_walls(current_cell, true_H, true_V, robot_H, robot_V)
    % Simulates sensing the walls of the current cell and updating the robot's internal map.
    r = current_cell(1);
    c = current_cell(2);
    % Sense and update North Wall info
    robot_H(r, c) = true_H(r, c);
    % Sense and update South Wall info
    robot_H(r + 1, c) = true_H(r + 1, c);
    % Sense and update West Wall info
    robot_V(r, c) = true_V(r, c);
    % Sense and update East Wall info
    robot_V(r, c + 1) = true_V(r, c + 1);
end
% -------------------------------------------------------------------------
function distance_grid = calculate_flood_fill(maze_size, end_cell, H_walls, V_walls)
    % Performs flood fill (a Breadth-First Search) on the robot's KNOWN map.
    distance_grid = inf(maze_size, maze_size);
    distance_grid(end_cell(1), end_cell(2)) = 0;
    q = {end_cell}; 
    while ~isempty(q)
        current_cell = q{1};
        q(1) = [];
        r = current_cell(1);
        c = current_cell(2);
        d = distance_grid(r, c);
        neighbors = [-1, 0; 1, 0; 0, -1; 0, 1]; 
        for i = 1:4
            nr = r + neighbors(i, 1);
            nc = c + neighbors(i, 2);
            if nr >= 1 && nr <= maze_size && nc >= 1 && nc <= maze_size
                is_wall = false;
                if i == 1 && H_walls(r, c) == 1, is_wall = true; end      
                if i == 2 && H_walls(r+1, c) == 1, is_wall = true; end  
                if i == 3 && V_walls(r, c) == 1, is_wall = true; end      
                if i == 4 && V_walls(r, c+1) == 1, is_wall = true; end  
                if ~is_wall && distance_grid(nr, nc) == inf
                    distance_grid(nr, nc) = d + 1;
                    q{end+1} = [nr, nc];
                end
            end
        end
    end
end
% -------------------------------------------------------------------------
function next_move = find_next_move(current_cell, distance_grid, H_walls, V_walls)
    % Decides the next cell to move to by finding the neighbor with the lowest distance value
    r = current_cell(1);
    c = current_cell(2);
    maze_size = size(distance_grid, 1);
    min_dist = inf;
    next_move = [];
    neighbors = [-1, 0; 1, 0; 0, -1; 0, 1];
    for i = 1:4
        nr = r + neighbors(i, 1);
        nc = c + neighbors(i, 2);
        if nr >= 1 && nr <= maze_size && nc >= 1 && nc <= maze_size
            is_wall = false;
            if i == 1 && H_walls(r, c) == 1, is_wall = true; end
            if i == 2 && H_walls(r+1, c) == 1, is_wall = true; end
            if i == 3 && V_walls(r, c) == 1, is_wall = true; end
            if i == 4 && V_walls(r, c+1) == 1, is_wall = true; end
            if ~is_wall && distance_grid(nr, nc) < min_dist
                min_dist = distance_grid(nr, nc);
                next_move = [nr, nc];
            end
        end
    end
end
% -------------------------------------------------------------------------
function animate_robot_movement(start_cell, end_cell, robot_handle, maze_size, is_reset)
    % Animates the robot's continuous movement.
    if nargin < 5
        is_reset = false;
    end
    start_xy = [start_cell(2)-0.5, maze_size - start_cell(1) + 0.5];
    end_xy = [end_cell(2)-0.5, maze_size - end_cell(1) + 0.5];
    move_vec = end_cell - start_cell;
    
    if all(move_vec == [-1, 0]), target_theta = pi/2;
    elseif all(move_vec == [1, 0]), target_theta = -pi/2;
    elseif all(move_vec == [0, 1]), target_theta = 0;
    elseif all(move_vec == [0, -1]), target_theta = pi;
    else, target_theta = 0;
    end
    
    base_shape = [-0.2, 0.3, -0.2; -0.25, 0, 0.25];
    
    if is_reset
        num_steps = 1; % Instantaneous jump for reset
    else
        num_steps = 20;
    end
    
    for i = 1:num_steps
        t = i / num_steps;
        current_xy = (1-t) * start_xy + t * end_xy;
        R = [cos(target_theta), -sin(target_theta); sin(target_theta), cos(target_theta)];
        rotated_shape = R * base_shape;
        set(robot_handle, 'XData', rotated_shape(1,:) + current_xy(1));
        set(robot_handle, 'YData', rotated_shape(2,:) + current_xy(2));
        drawnow;
    end
end