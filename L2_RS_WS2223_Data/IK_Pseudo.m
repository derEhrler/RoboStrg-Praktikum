function out = IK_Pseudo(q_ist,position_soll)



%% Do 128 iterations to estimate the inverse kinematic
for i=0:127
    % Get jacobian for position. This function should be programmed in the
    % task before
    % Caution: Rad/Degree!
    % J = Jacobi_Matrix(q);
    
    % Get transformation matrix
    % Caution: Rad/Degree!
    % T = TransformationsMatrix(q);
    
    % Calculate the error
  
    % Calculation step with pseudo inverse jacobian
    
end

out = q;
end