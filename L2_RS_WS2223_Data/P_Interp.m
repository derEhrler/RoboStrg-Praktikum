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
t1 = round(t1,"minusinf");
p1 = p0 + directionP * 0.5 * a * t1^2;


%Nullbeschleunigung
p2 = P_soll - directionP * v/a;
t2 = t1 + (p2 - p1) / v;
t2 = round(t2,"minusinf");

%Zielpunkt
pn = P_soll;
tn = t2 + v/a;
tn = round(tn,"minusinf");

%time steps
noSteps = tn / dt;
noSteps = round(noSteps,"minusinf");
P = zeros(noSteps,3);

%interpolating for acceleration phase
for i=0:dt:t1
    P(i,:) = 
end









out = P;
end