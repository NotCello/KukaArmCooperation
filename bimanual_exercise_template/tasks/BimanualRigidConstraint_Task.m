classdef BimanualRigidConstraint_Task < Task
    properties
        % The "Invisible Sticks" (Constant Relative Transforms)
        tL_T_o % Transform: Left Tool -> Object
        tR_T_o % Transform: Right Tool -> Object
        
        wTog   % Final Goal for the Object
    end
    
    methods
        function obj = BimanualRigidConstraint_Task(tL_T_o, tR_T_o, wTog, taskID)
            obj.ID = 'BM'; % Bi-Manual Task
            obj.task_name = taskID;
            obj.tL_T_o = tL_T_o;
            obj.tR_T_o = tR_T_o;
            obj.wTog = wTog;
        end
        
        function updateReference(obj, robot_system)
            % 1. Where is the object NOW? (Compute using Left Arm)
            % wTo = wTt * tTo
            wTo_current = robot_system.left_arm.wTt * obj.tL_T_o;
            
            % 2. Error: Where should the object BE?
            [v_ang, v_lin] = CartError(obj.wTog, wTo_current);
            
            % 3. Velocity Reference
            v_ref = [v_ang; v_lin];
            v_ref = Saturate(v_ref, 0.2); % Safety limit
            
            % We want the OBJECT to move at v_ref.
            % Since we control the object twice (Left chain, Right chain),
            % we stack the reference:
            obj.xdotbar = [v_ref; v_ref]; 
        end
        
        function updateJacobian(obj, robot_system)
            % --- LEFT ROBOT ---
            % 1. Get the "Lever Arm" (Vector r from Tool to Object)
            % We use the orientation of the tool to rotate the relative vector into world frame
            r_L = obj.tL_T_o(1:3,4); % Position offset in Tool Frame
            w_r_L = robot_system.left_arm.wTt(1:3,1:3) * r_L; % Rotate to World Frame
            
              % MATRIX S (Rigid Body Transform)
            
            S_L = [eye(3), zeros(3); -skew(w_r_L), eye(3)];
            
            % 3. Transform the Tool Jacobian to Object Jacobian
            % J_object = S * J_tool
            J_L_obj = S_L * robot_system.left_arm.wJt;
            
            
            % --- RIGHT ROBOT ---
            % 1. Lever Arm
            r_R = obj.tR_T_o(1:3,4);
            w_r_R = robot_system.right_arm.wTt(1:3,1:3) * r_R;
            
            % 2. COMPUTE MATRIX S
            S_R = [eye(3), zeros(3); -skew(w_r_R), eye(3)];
            
            % 3. Object Jacobian
            J_R_obj = S_R * robot_system.right_arm.wJt;
            
            
            % --- COMBINE ---
            % Block Diagonal Jacobian (12 x 14)
            obj.J = blkdiag(J_L_obj, J_R_obj);
        end
        
        function updateActivation(obj, robot_system)
            % Binary transition (Always active in Phase 2)
            obj.A = eye(12);
        end
    end
end