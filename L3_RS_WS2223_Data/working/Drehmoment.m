function M = Drehmoment(q,g,m_Link,m_Motor,JL1,JL2,JL3,JL4,JM1,JM2,JM3,JM4)
%q: Gelenkwinkel
%g: Gravitationsvektor

for i=1:4
    %Rufen Sie hier die Jacobi-Matrizen als Funktion auf, die Sie in P_3.m
    %erzeugt haben
    
     M(i)=-(m_Link*[g*JL1(:,i);g*JL2(:,i);g*JL3(:,i);g*JL4(:,i)]+m_Motor*[g*JM1(:,i);g*JM2(:,i);g*JM3(:,i);g*JM4(:,i)]);
    
    
end

end