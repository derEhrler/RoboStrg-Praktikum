function out = JacobiMatrix(T)
%% Bitte vervollständigen 
% Calculate the jacobian with the transformation matrices
% T should be a 3 dimensional matrix, where e.g. T(:,:,1) ist the
% transformation matrix of the first joint.
% First, calculate each column j_i of the jacobian seperately with the function JacobiSpalte and then add
% them to a matrix J = [j_1 j_2 j_3 ...].
A_0 = eye(4,4);
j_1 = JacobiSpalte(A_0(:,:),T(:,:,4));
j_2 = JacobiSpalte(T(:,:,1),T(:,:,4));
j_3 = JacobiSpalte(T(:,:,2),T(:,:,4));
j_4 = JacobiSpalte(T(:,:,3),T(:,:,4));

%j_1 = -j_1;
J = [j_1 j_2 j_3 j_4];


%% Important
% After calculating j_1, please use the negative value of the first column: j_1 = -j_1

out = J;
end