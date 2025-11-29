classdef BimanualRigidConstraint_Task < Task
    % Task to move an object held rigidly by two robots
    
    properties
        % Relative transforms (Fixed once grasped)
        tL_T_o % Transform from Left Tool to Object
        tR_T_o % Transform from Right Tool to Object
        
        wTog   % Goal Frame for the Object
    end
    
    methods
        % Constructor
        function obj = BimanualRigidConstraint_Task(tL_T_o, tR_T_o, wTog, taskID)
            obj.ID = 'BM'; % Bi-Manual
            obj.task_name = taskID;
            obj.tL_T_o = tL_T_o;
            obj.tR_T_o = tR_T_o;
            obj.wTog = wTog;
        end
        
        function updateReference(obj, robot_system)
            % 1. Compute current Object Frame (wTo)
            % We can estimate it using the Left Arm and the fixed offset
            wTo_current = robot_system.left_arm.wTt * obj.tL_T_o;
            
            % 2. Compute Error (Goal Object Frame vs Current Object Frame)
            [v_ang, v_lin] = CartError(obj.wTog, wTo_current);
            
            % 3. Define Reference Velocity (12 Dimensions)
            % We want BOTH arms to move the object to the goal.
            % So we stack the object velocity request twice.
            
            v_ref = 1.0 * [v_ang; v_lin];
            
            % Saturate for safety
            v_ref(1:3) = Saturate(v_ref(1:3), 0.3);
            v_ref(4:6) = Saturate(v_ref(4:6), 0.3);
            
            obj.xdotbar = [v_ref; v_ref]; % 12x1 Vector
        end
        
        function updateJacobian(obj, robot_system)
            % 1. Get Base Jacobians
            JL = robot_system.left_arm.wJt;
            JR = robot_system.right_arm.wJt;
            
            % 2. Calculate Lever Arms (Vector from Tool to Object)
            % w_r_to = w_pos_object - w_pos_tool
            % Note: wTo = wTt * tTo
            
            wTo_L = robot_system.left_arm.wTt * obj.tL_T_o;
            w_r_L = wTo_L(1:3,4) - robot_system.left_arm.wTt(1:3,4);
            
            wTo_R = robot_system.right_arm.wTt * obj.tR_T_o;
            w_r_R = wTo_R(1:3,4) - robot_system.right_arm.wTt(1:3,4);
            
            % 3. Compute Rigid Body Transformation Matrices
            % [ I   0 ]
            % [ -S  I ]
            XL = [eye(3), zeros(3); -skew(w_r_L), eye(3)];
            XR = [eye(3), zeros(3); -skew(w_r_R), eye(3)];
            
            % 4. Compute Object Jacobians for each arm
            J_obj_L = XL * JL;
            J_obj_R = XR * JR;
            
            % 5. Build Block Diagonal Jacobian (12 x 14)
            % [ J_obj_L     0     ]
            % [    0     J_obj_R  ]
            obj.J = blkdiag(J_obj_L, J_obj_R); 
        end
        
        function updateActivation(obj, robot_system)
            % Binary transition: Always Active (1)
            obj.A = eye(12);
        end
    end
end