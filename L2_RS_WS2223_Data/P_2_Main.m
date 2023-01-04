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
P_start = [0.0 0.0 0.0]';
P_soll = [0.41 0.2595 0.084]';

%P = linear_Interpol(P_start, P_soll, Zwischenschritte);

%% Erweiterung mit Geschw. und Beschleunigung
v = 0.2;
a = 0.5;
[P,testValue] = P_Interp(P_start, P_soll, v, a);

plot(P(:,1));

