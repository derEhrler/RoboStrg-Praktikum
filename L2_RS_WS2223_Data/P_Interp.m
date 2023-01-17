function [out,testValue] = P_Interp(P_start, P_soll, v, a)
 
%initial calculations
dt = 0.01;
directionP = P_soll - P_start;
distanceP = norm(directionP);
directionP = directionP / norm(directionP);


%Startpunkt
t0 = 0;
p0 = P_start;

%Beschleunigungsphase
t1 = v/a;
p1 = p0 + directionP * 0.5 * a * t1^2;

%Nullbeschleunigung
p2 = P_soll - directionP * 0.5 * a * (v/a)^2;
distA0 = p2 - p1;
t2 = t1 + norm(distA0) / v;

%check for short distance
vecP1P2 = p2 - p1;
shortDist = false;
if dot(vecP1P2,directionP) < 0
    shortDist = true;
end

%% long distance
if shortDist == false
    disp('Distance is long');

    %Zielpunkt
    pn = P_soll;
    tn = t2 + v/a;
    
    %time steps
    noSteps = tn / dt;
    noSteps = round(noSteps);
    P = zeros(noSteps,3);
    V = zeros(noSteps,3);
    
    %interpolating for acceleration phase
    currIndex = 1;
    for i = 0:dt:t1
        P(currIndex,:) = p0' + directionP' * 0.5 * a * i^2;
        V(currIndex,:) = directionP' * a * i;
        currIndex = currIndex + 1;
    end
    
    %interpolating for cruising phase
    for i = t1:dt:t2
        P(currIndex,:) = p1' + directionP' * v * (i - t1);
        V(currIndex,:) = directionP' * v;
        currIndex = currIndex + 1;
    end
    
    %interpolating for breaking phase
    for i = t2:dt:tn
        P(currIndex,:) = p2' + directionP' * v * (i - t2) - directionP' * 0.5 * a * (i - t2)^2; 
        V(currIndex,:) = directionP' * v - directionP' * a * (i - t2);
        currIndex = currIndex + 1;
    end
end

%% short distance
if shortDist == true
    disp('Distance is short');

    %time, position, velocity
    t1kurz = sqrt((distanceP/2)*2 / a);
    tnkurz = 2 * t1kurz;
    p1 = p0 + directionP * 0.5 * a * t1kurz^2;
    vkurz = a * (t1kurz - t0);

    %time steps
    noSteps = tnkurz / dt;
    noSteps = round(noSteps);
    P = zeros(noSteps,3);
    V = zeros(noSteps,3);

    %interpolating for acceleration phase
    currIndex = 1;
    for i = 0:dt:t1kurz
        P(currIndex,:) = p0' + directionP' * 0.5 * a * i^2;
        V(currIndex,:) = directionP' * a * i;
        currIndex = currIndex + 1;
    end

    %interpolating for breaking phase
    for i = t1kurz:dt:tnkurz
        P(currIndex,:) = p1' + directionP' * vkurz * (i - t1kurz) - directionP' * 0.5 * a * (i - t1kurz)^2; 
        V(currIndex,:) = directionP' * vkurz - directionP' * a * (i - t1kurz);
        currIndex = currIndex + 1;
    end
end


hold on;
plot(P(:,1));
plot(P(:,2));
plot(P(:,3));
plot(V(:,1));
plot(V(:,2));
plot(V(:,3));
hold off;


testValue = p2;
out = P;
end