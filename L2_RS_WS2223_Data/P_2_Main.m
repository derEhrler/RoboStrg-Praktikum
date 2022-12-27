%% Gelenkwinkel:
% Je nach Gelenkstellung bitte ändern.
q = [180 270 90 180 180 0]'; 

%% erzeuge Transformationsmatrizen
T = TransFormMatrix(q);

%% Jacbobi-Matrix 
J = JacobiMatrix(T);

%% Inverse Kinematik 
%q_soll_Transp = IK_Transp(q,[x y z]);
%q_soll_Pseudo = IK_Pseudo(q,[x y z]);
