r_Link = zeros(4,4);
r_Motor = zeros(4,4);
m_Link = zeros(1,4);
m_Motor = zeros(1,4);
M = zeros(1,4);
% Roboter Parameter
D1 = 0.2755;
D2 = 0.410;
D3 = 0.2073;
D4 = 0.0741;
D6 = 0.160;
e2 = 0.0098;
d6b = D3+D6;

%q=sym('q',[4,1]); % F�r die symbolische Berechnung
q=[180 120 180 40];
g=[0 0 -9.81];
% DH-Parameter: Roboter
d(1) = D1;   a(1) = 0;  alpha(1) = pi/2;  theta(1) = q(1);   
d(2) = 0;    a(2) = D2; alpha(2) = pi;      theta(2) = q(2); 
d(3) = -e2;  a(3) = 0;  alpha(3) = pi/2;    theta(3) = q(3); 
d(4) = -d6b;  a(4) = 0;   alpha(4) = pi;    theta(4) = q(4); 

DH_Parameter = [ q(1)*pi/180    D1     0     pi/2;
                 q(2)*pi/180    0      D2    pi;
                 q(3)*pi/180    -e2    0     pi/2;
                 q(4)*pi/180    -d6b   0     pi ];
% Position: Schwerpunkt 
r_Link(:,1) = [     0;     -50;         0;         1000]/1000;
r_Link(:,2) = [  -200;       0;         0;         1000]/1000;
r_Link(:,3) = [     0;       0;       -50;         1000]/1000;
r_Link(:,4) = [     20;       0;       -50;         1000]/1000;

r_Motor(:,1) = [  0;       0;       150;         1000]/1000;
r_Motor(:,2) = [  0;       0;          0;         1000]/1000;
r_Motor(:,3) = [  0;       0;          0;         1000]/1000;
r_Motor(:,4) = [  0;       0;     -200;         1000]/1000;

% Masse: Link
m_Link(1) = (200)/1000;
m_Link(2) = (400)/1000;
m_Link(3) = (200)/1000;
m_Link(4) = (700)/1000;

% Masse:Motor
m_Motor(1) = 500/1000;
m_Motor(2) = 500/1000;
m_Motor(3) = 500/1000;
m_Motor(4) = 300/1000;


%Beachten Sie, dass die Schwerpunkte der Motoren nicht im Ursprung der
%Koordinatensysteme befinden
%Tipp: Nachdem Sie die Zeichnung angefertigt haben, �berlegen Sie 
%sich welcher Link-Vektor oder Motor-Vektor zu welcher 
%Transformationsmatrix geh�rt.

%Tipp2: Der erste Link-Vektor bezieht sich auf T0*T1. Der erste 
%Motor-Vektor bezieht sich auf T0. Es ist daher hilfreich neue 
%Transformationsmatrizen f�r Link und Motor zu erstellen
T=Transformationsmatrix(q);
%Transformationsmatrizen fuer Links
PL1=T(:,:,1)*r_Link(:,1);
PL2=T(:,:,2)*r_Link(:,2);
PL3=T(:,:,3)*r_Link(:,3);
PL4=T(:,:,4)*r_Link(:,4);
%Transformationsmatrizen fuer Motoren
PM1=eye(4)*r_Motor(:,1);
PM2=T(:,:,1)*r_Motor(:,2);
PM3=T(:,:,2)*r_Motor(:,3);
PM4=T(:,:,3)*r_Motor(:,4);



%Erzeugen Sie die einzelnen Jacobi-Matrizen und speichern Sie diese als
%MATLAB-Funktion ab.
z0=[0 0 1]';
z1=T(1:3,3,1);
z2=T(1:3,3,2);
z3=T(1:3,3,3);
P0=[0 0 0]';
P1=T(1:3,4,1);
P2=T(1:3,4,2);
P3=T(1:3,4,3);
%Jacobi_Matrix Link 1-4:
JL1=[cross(z0,PL1(1:3)-P0)  zeros(3,3)];                  
JL2=[cross(z0,PL2(1:3)-P0)  cross(z1,PL2(1:3)-P1) zeros(3,2)];
JL3=[cross(z0,PL3(1:3)-P0)  cross(z1,PL3(1:3)-P1) cross(z2,PL3(1:3)-P2) zeros(3,1)];
JL4=[cross(z0,PL4(1:3)-P0)  cross(z1,PL4(1:3)-P1) cross(z2,PL4(1:3)-P2) cross(z3,PL4(1:3)-P3)];
%Jacobi_Matrix Motor 1-4:
JM1=[cross(z0,PM1(1:3)-P0)  zeros(3,3)];
JM2=[cross(z0,PM2(1:3)-P0)  cross(z1,PM2(1:3)-P1) zeros(3,2)];
JM3=[cross(z0,PM3(1:3)-P0)  cross(z1,PM3(1:3)-P1) cross(z2,PM3(1:3)-P2) zeros(3,1)];
JM4=[cross(z0,PM4(1:3)-P0)  cross(z1,PM4(1:3)-P1) cross(z2,PM4(1:3)-P2) cross(z3,PM4(1:3)-P3)];


M=Drehmoment(q,g,m_Link,m_Motor,JL1,JL2,JL3,JL4,JM1,JM2,JM3,JM4);