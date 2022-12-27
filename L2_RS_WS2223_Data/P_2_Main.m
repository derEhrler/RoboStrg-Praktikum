%% Gelenkwinkel:
% Je nach Gelenkstellung bitte ändern.
q = [180 270 90 180 180 0]'; 

%% Robot parameter (Do not modify!)
D = [0.2755; 0.41; 0.2073; 0.0741; 0.0741; 0.16];
e2 = 0.0098;

aa = deg2rad(30);
ca = cos(aa);
sa = sin(aa);
c2a = cos(2*aa);
s2a = sin(2*aa);
d4b = D(3)+sa/s2a*D(4);
d5b = sa/s2a*D(4)+sa/s2a*D(5);
d6b = sa/s2a*D(5)+D(6);

%% DH parameter (Do not modify!): [theta d a alpha] . Winkel sind in RAD und Positionen in Meter


DH_Parameter = [ -q(1)*pi/180      D(1)  0    pi/2;...
                 (q(2)-90)*pi/180  0     D(2) pi;...
                 (q(3)+90)*pi/180  -e2   0    pi/2;...
                  q(4)*pi/180      -d4b  0    2*aa;...
                 (q(5)-180)*pi/180 -d5b  0    2*aa;...
                 (q(6)+90)*pi/180  -d6b  0    pi ];

%% Erzeuge Transformationsmatrizen
A0 = [1 0 0 0
      0 1 0 0
      0 0 1 0
      0 0 0 1];
A1 = DH(DH_Parameter(1,:));
A2 = DH(DH_Parameter(2,:));
A3 = DH(DH_Parameter(3,:));
A4 = DH(DH_Parameter(4,:));
A5 = DH(DH_Parameter(5,:));
A6 = DH(DH_Parameter(6,:));

%% Transformation Hand-Roboter
% Bitte vervollständigen

T = zeros(4,4,6);

T(:,:,1) = A0*A1;
T(:,:,2) = T(:,:,1)*A2;
T(:,:,3) = T(:,:,2)*A3;
T(:,:,4) = T(:,:,3)*A4;
T(:,:,5) = T(:,:,4)*A5;
T(:,:,6) = T(:,:,5)*A6;
%T = ...
% Tipp: Kopieren Sie den vorherigen Code und erzeugen Sie eine Funktion mit
% T = TransformatrionsMatrix(q) mit den Übergabeparametern Gelenkwinkel und
% als Rückgabewert die Transformationsmatrizen.

%% Jacbobi-Matrix 

J = JacobiMatrix(T);
%% Inverse Kinematik 
%q_soll_Transp = IK_Transp(q,[x y z]);
%q_soll_Pseudo = IK_Pseudo(q,[x y z]);
