function x = mulpq( p, q )
	
s1 = p(1);
v1 = p(2:4);

s2 = q(1);
v2 = q(2:4);

x = [  s1*s2 - v1'*v2;  s1*v2 + s2*v1 + cross( v1, v2 )  ];

