function out = IK_Pseudo(q_ist,position_soll)



%% Do 128 iterations to estimate the inverse kinematic
q = q_ist;
q_prev = q_ist;
for i=0:127
    % Get jacobian for position. This function should be programmed in the
    % task before
    % Caution: Rad/Degree!
    % J = Jacobi_Matrix(q);
    % Get transformation matrix
    % Caution: Rad/Degree!
    % T = Transformations_Matrix(q);
    T = TransFormMatrix(q);
    J = JacobiMatrix(T);
    
    % Caclulate the error
    e_x = position_soll(1,1) - T(1,4,6);
    e_y = position_soll(1,2) - T(2,4,6);
    e_z = position_soll(1,3) - T(3,4,6);
    e = [e_x,e_y,e_z]';
    if(norm(e) < 0.01)
        disp('Position error < 0.01m');
        disp(i);
        break;
    end

    % Calculation step with transpose jacobian
    K = 15;
    MAX_ANGLE_STEP = 2;

    J_pseudo = transpose(J) * inv(J * transpose(J));
    q = q + K * J_pseudo * e;

    for j=1:6
        if(abs(q_prev(j))-abs(q(j)) > MAX_ANGLE_STEP)
            disp('ERROR: Step was to big.');
            break;
        end
    end
    q_prev = q;
end

out = q;
end