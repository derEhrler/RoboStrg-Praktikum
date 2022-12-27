function out = JacobiSpalte(T_i,T_tcp)
%% Bitte vervollständigen 
% Calculate the collumn of the jacobian for joint i

J_1 = cross(T_i(1:3,3),(T_tcp(1:3,4)-T_i(1:3,4)));
J_2 = T_i(1:3,3);
%J = [J_1;
%     J_2];
J = J_1;

out = J;
end