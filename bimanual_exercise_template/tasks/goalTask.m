classdef goalTask < Task   
    %Tool position control for a single arm
    properties

    end

    methods
        function obj=goalTask(robot_ID,taskID)
            obj.ID=robot_ID;
            obj.task_name=taskID;
        end
        function updateReference(obj, robot_system)
            % 1. Define Object Parameters
            w_O_o = [0.5; 0; 0.59]; 
            l_obj = 0.12; 
        
            % 2. Select Robot and Define Fixed Nominal Orientation (R_base)
            % We define R_base so the gripper (Z-axis) points toward the object center
            if obj.ID == 'L'
              robot = robot_system.left_arm;
            
              % Left arm is at x=0, Object at x=0.5. Gripper must point +X.
              % We rotate the World Frame (Z-up) 90 deg around Y to point Z along X.
              R_base = [0 0 1; 0 1 0; -1 0 0];
            
              % Grasp the LEFT side (negative offset along X relative to object center)
               w_P_L= [0; -l_obj/2; 0]; 

            elseif obj.ID == 'R'
              robot = robot_system.right_arm;
            
              % Right arm is at x=1.06, Object at x=0.5. Gripper must point -X.
              % We rotate World Frame -90 deg around Y.
              R_base = [0 0 -1; 0 1 0; 1 0 0];
            
              % Grasp the RIGHT side (positive offset along X)
        
              w_P_L= [0; l_obj/2; 0]; 
            end

    % 3. Calculate Goal Position (Vector addition)
    goal_pos = w_O_o + w_P_L;

    % 4. Compute Final Goal Orientation (Rotate 30 deg around Y)
    % Use the provided rotation function: rotation(x, y, z) in radians
    R_tilt = rotation(0, deg2rad(30), 0); 
    R_goal = R_base * R_tilt;
    
    % 5. Construct the Goal Transformation Matrix
    wTg = [R_goal, goal_pos; 0 0 0 1];


end
        function updateActivation(obj, robot_system)
            obj.A = eye(6);
        end
    end
end