function [ u, theta, R, t ] = dualq2uthetaRt( x )
	
qrr = x(1:4);
qdd = x(5:8);

t = 2*mulpq( qdd, conjq( qrr ) );
t = t(2:4);
 
 
theta = 2*acos( qrr(1) );

if(theta ~= 0) 
u = qrr(2:4) / sin(theta/2) ;
else
u = [0; 0; 1];
end

skw = [ 0, -u(3),  u(2);  
             u(3),  0,   -u(1);
            -u(2),  u(1),  0];
            
            
            
 R = eye(3)  + sin(theta)*skw + skw*skw*( 1 - cos(theta) );
 
 