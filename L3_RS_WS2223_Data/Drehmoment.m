function out = Drehmoment(q,g,J_l,J_m,m_Link,m_Motor)
%q: Gelenkwinkel
%g: Gravitationsvektor
%J_l: Jacobian Links
%J_m: Jacobian Motoren
M = zeros(1,4);

%Gleichung 1.2
for i=1:4
    M(1,i) = - (m_Link(i)*g'*J_l(1:6,i) + m_Motor(i)*g'*J_m(1:6,i));
    %Rufen Sie hier die Jacobi-Matrizen als Funktion auf, die Sie in P_3.m
    %erzeugt haben
    
end
out = M;
end