function [out,testValue] = P_Interp(P_start, P_soll, v, a)
    
dt = 0.01;
directionP = P_soll - P_start;
distanceP = norm(directionP);
directionP = directionP / norm(directionP);


%Startpunkt
t0 = 0;
p0 = P_start;

%Beschleunigungsphase
t1 = v/a;
%t1 = round(t1,TieBreaker="minusinf");
p1 = p0 + directionP * 0.5 * a * t1^2;


%Nullbeschleunigung
p2 = P_soll - directionP * 0.5 * a * (v/a)^2;
distA0 = p2 - p1;
t2 = t1 + norm(distA0) / v;
%t2 = round(t2,TieBreaker="minusinf");

%Zielpunkt
pn = P_soll;
tn = t2 + v/a;
%tn = round(tn,TieBreaker="minusinf");

%time steps
noSteps = tn / dt;
noSteps = round(noSteps,TieBreaker="minusinf");
P = zeros(noSteps,3);
V = zeros(noSteps,3);

%interpolating for acceleration phase
currIndex = 1;
for i = 0:dt:t1
    P(currIndex,:) = directionP' * 0.5 * a * i^2;
    V(currIndex,:) = a * i;
    currIndex = currIndex + 1;
end

for i = t1:dt:t2
    P(currIndex,:) = p1' + directionP' * v * (i - t1);
    V(currIndex,:) = v;
    currIndex = currIndex + 1;
end

for i = t2:dt:tn
    P(currIndex,:) = p2' + directionP' * v * (i - t2) - directionP' * 0.5 * a * (i - t2)^2; 
    V(currIndex,:) = v - a * (i - t2);
    currIndex = currIndex + 1;
end


hold on;
plot(P(:,1));
plot(P(:,2));
plot(P(:,3));
plot(V(:,1));
hold off;


testValue = V;
out = P;
end