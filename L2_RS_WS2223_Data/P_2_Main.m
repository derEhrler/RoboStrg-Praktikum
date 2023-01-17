%% Gelenkwinkel:
% Je nach Gelenkstellung bitte �ndern.
q = [150 270 90 180 180 0]';
x = 0.41;
y = 0.2595;
z = 0.084;

%% erzeuge Transformationsmatrizen
T = TransFormMatrix(q);

%% Jacbobi-Matrix 
J = JacobiMatrix(T);

%% Inverse Kinematik 
%q_soll_Transp = IK_Transp(q,[x y z]);
%q_soll_Pseudo = IK_Pseudo(q,[x y z]);

%% Interpolation
q_start = [275.35 167.48 57.08 -119.11 82.77 75.68]';
tmp = TransFormMatrix(q_start);
P_start = [tmp(1,4,6), tmp(2,4,6), tmp(3,4,6)]';
P_soll = [0.41 0.2595 0.084]';
v = 0.1;
a = 1;

%Zwischenschritte = 128;
%P = linear_Interpol(P_start, P_soll, Zwischenschritte);

[P,testValue] = P_Interp(P_start, P_soll, v, a);

%% Inverse Kinematik für alle P
P_IK_Matrix = zeros(length(P(:,1)),6);
P_IK_Matrix(1,:) = IK_Pseudo(q_start,P_start');
for i = 2:length(P(:,1))
    P_IK_Matrix(i,:) = IK_Pseudo(P_IK_Matrix(i-1,:)',P(i,:))';
end

%% Werte in ein File lesen
fid = fopen( 'P_Matrix.txt', 'wt' );
for i = 1:length(P(:,1))
  fprintf(fid, '%f %f %f %f %f %f\n' , P_IK_Matrix(i,1), P_IK_Matrix(i,2), P_IK_Matrix(i,3), P_IK_Matrix(i,4), P_IK_Matrix(i,5), P_IK_Matrix(i,6));
end
fclose(fid);


