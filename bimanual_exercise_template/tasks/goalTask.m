classdef goalTask < Task   
    % Task to drive the end-effector to the goal frame (robot.wTg)
    
    properties
    end
    
    methods
        % Constructor
        function obj = goalTask(robot_ID, taskID)
            obj.ID = robot_ID;
            obj.task_name = taskID;
        end
        
        % 1. Update Reference: Calculate velocity to move wTt -> wTg
        function updateReference(obj, robot_system)
            % Select the correct robot arm
            if obj.ID == 'L'
                robot = robot_system.left_arm;
            elseif obj.ID == 'R'
                robot = robot_system.right_arm;
            end
            
            % Compute the error between the Goal Frame (wTg) and Current Frame (wTt)
            % Note: robot.wTg is set in main.m via setGoal()
            [v_ang, v_lin] = CartError(robot.wTg, robot.wTt);
            
            % Set the reference velocity
            obj.xdotbar = [v_ang; v_lin];
            
            % Saturate limits to prevent instability
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.5); % Angular limit
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.3); % Linear limit
        end
        
        % 2. Update Jacobian: Map task velocities to joint velocities
        function updateJacobian(obj, robot_system)
            if obj.ID == 'L'
                robot = robot_system.left_arm;
            elseif obj.ID == 'R'
                robot = robot_system.right_arm;
            end
            
            % Use the geometric Jacobian of the tool
            tool_jacobian = robot.wJt;
            
            % Construct the full system Jacobian (14 columns: 7 Left + 7 Right)
            if obj.ID == 'L'
                obj.J = [tool_jacobian, zeros(6, 7)];
            elseif obj.ID == 'R'
                obj.J = [zeros(6, 7), tool_jacobian];
            end
        end
        
        % 3. Update Activation: Always active (Identity matrix)
        function updateActivation(obj, robot_system)
            obj.A = eye(6);
        end
    end
end