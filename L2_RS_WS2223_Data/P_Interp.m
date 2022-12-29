function out = P_Interp(P_start, P_soll, v, a)
    
dt = 0.01;

%Startpunkt
t0 = 0;
p0 = P_start;

%Beschleunigungsphase
t1 = v/a;
p1 = 0.5 * a * t1^2;

%Nullbeschleunigung


%Zielpunkt
pn = P_soll;
tn = t2 + v/a;








out = P;
end