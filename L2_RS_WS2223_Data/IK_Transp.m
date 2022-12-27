function out = IK_Transp(q_ist,position_soll)



%% Do 128 iterations to estimate the inverse kinematic
q = q_ist;
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

    % Calculation step with transpose jacobian
    K = 1;
    q = q + K * transpose(J) * e;
end

out = q;
end