function out = Transformationsmatrix(q)
D1 = 0.2755;
D2 = 0.410;
D3 = 0.2073;
D4 = 0.0741;
D6 = 0.160;
e2 = 0.0098;
d6b = D3+D6;
DH_Parameter = [ q(1)*pi/180    D1     0     pi/2;
                 q(2)*pi/180    0      D2    pi;
                 q(3)*pi/180    -e2    0     pi/2;
                 q(4)*pi/180    -d6b   0     pi ];
             
A0 = [1 0 0 0
      0 1 0 0
      0 0 1 0
      0 0 0 1];
A1 = DH(DH_Parameter(1,:));
A2 = DH(DH_Parameter(2,:));
A3 = DH(DH_Parameter(3,:));
A4 = DH(DH_Parameter(4,:));

T = zeros(4,4,4);

T(:,:,1) = A0*A1;
T(:,:,2) = T(:,:,1)*A2;
T(:,:,3) = T(:,:,2)*A3;
T(:,:,4) = T(:,:,3)*A4;
out=T;
end