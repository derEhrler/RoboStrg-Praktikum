function [ T ] = DH(dh_param )

theta = dh_param(1);
d     = dh_param(2);
a     = dh_param(3);
alpha = dh_param(4);

T = [ 
    cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)    a*cos(theta);
    sin(theta) cos(theta)*cos(alpha)  -cos(theta)*sin(alpha)    a*sin(theta);
    0          sin(alpha)             cos(alpha)            	d           ;
    0          0                      0                         1]          ;

end
