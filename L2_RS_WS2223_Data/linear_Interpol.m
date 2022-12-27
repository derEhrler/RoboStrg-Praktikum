function out = linear_Interpol(P_start, P_soll, Zwischenschritte)
    
    P = zeros(Zwischenschritte,3);
    deltaP = P_soll - P_start;
    t = linspace(0,1,Zwischenschritte);

    for i=1:Zwischenschritte
        P(i,:) = P_start(:,1)' + t(i) * deltaP(:,1)';
    end

    out = P;
end