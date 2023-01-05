%% Gelenkwinkel:
% Je nach Gelenkstellung bitte ändern.
q = [150 270 90 180 180 0]';
x = 0.41;
y = 0.2595;
z = 0.084;

%% erzeuge Transformationsmatrizen
T = TransFormMatrix(q);

%% Jacbobi-Matrix 
J = JacobiMatrix(T);

%% Inverse Kinematik 
q_soll_Transp = IK_Transp(q,[x y z]);
q_soll_Pseudo = IK_Pseudo(q,[x y z]);

%% lineare Interpolation
Zwischenschritte = 128;
P_start = [0.1 0.2 0.3]';
P_soll = [0.41 0.2595 0.084]';

%P = linear_Interpol(P_start, P_soll, Zwischenschritte);

%% Erweiterung mit Geschw. und Beschleunigung
v = 0.2;
a = 0.5;
[P,testValue] = P_Interp(P_start, P_soll, v, a);

%% Inverse Kinematik für alle P
P_IK_Matrix = zeros(length(P(:,1)),6);
P_IK_Matrix(1,:) = IK_Pseudo(q,[x y z]);
for i = 2:length(P(:,1))
    P_IK_Matrix(i,:) = IK_Pseudo(P_IK_Matrix(i-1,:)',P(i,:))';
end

%% Werte in ein File lesen
fid = fopen( 'P_Matrix.txt', 'wt' );
for i = 1:length(P(:,1))
  fprintf(fid, '%f %f %f\n' , P(i,1), P(i,2), P(i,3));
end
fclose(fid);


