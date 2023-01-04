function out = P_Interp(P_start, P_soll, v, a)
    
dt = 0.01;
directionP = P_soll - P_start;
distanceP = norm(directionP);
directionP = directionP / distanceP;


%Startpunkt
t0 = 0;
p0 = P_start;

%Beschleunigungsphase
t1 = v/a;
p1 = p0 + directionP * 0.5 * a * t1^2;


%Nullbeschleunigung
p2 = P_soll - directionP * v/a;
t2 = t1 + (p2 - p1) / v;

%Zielpunkt
pn = P_soll;
tn = t2 + v/a;

%time steps
steps = 0:dt:tn;









out = P;
end