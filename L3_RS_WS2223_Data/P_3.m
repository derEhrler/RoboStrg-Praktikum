% Roboter Parameter
D1 = 0.2755;
D2 = 0.410;
D3 = 0.2073;
D4 = 0.0741;
D6 = 0.160;
e2 = 0.0098;
d6b = D3+D6;

%q=sym('q',[4,1]); % Für die symbolische Berechnung

% DH-Parameter: Roboter
d(1) = D1;   a(1) = 0;  alpha(1) = pi/2;  theta(1) = q(1);   
d(2) = 0;    a(2) = D2; alpha(2) = pi;      theta(2) = q(2); 
d(3) = -e2;  a(3) = 0;  alpha(3) = pi/2;    theta(3) = q(3); 
d(4) = -d6b;  a(4) = 0;   alpha(4) = pi;    theta(4) = q(4); 

% Position: Schwerpunkt 
r_Link(:,1) = [     0;     -50;         0]/1000;
r_Link(:,2) = [  -200;       0;         0]/1000;
r_Link(:,3) = [     0;       0;       -50]/1000;
r_Link(:,4) = [     20;       0;       -50]/1000;

r_Motor(:,1) = [  0;       0;       150]/1000;
r_Motor(:,2) = [  0;       0;          0]/1000;
r_Motor(:,3) = [  0;       0;          0]/1000;
r_Motor(:,4) = [  0;       0;     -200]/1000;

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
%Tipp: Nachdem Sie die Zeichnung angefertigt haben, überlegen Sie 
%sich welcher Link-Vektor oder Motor-Vektor zu welcher 
%Transformationsmatrix gehört.

%Tipp2: Der erste Link-Vektor bezieht sich auf T0*T1. Der erste 
%Motor-Vektor bezieht sich auf T0. Es ist daher hilfreich neue 
%Transformationsmatrizen für Link und Motor zu erstellen


%Transformationsmatrizen fuer Links

%Transformationsmatrizen fuer Motoren



%Erzeugen Sie die einzelnen Jacobi-Matrizen und speichern Sie diese als
%MATLAB-Funktion ab.

%Jacobi_Matrix Link 1-4:

%Jacobi_Matrix Motor 1-4:


