function x = muldualpq( p, q )
	
p1 = p(1:4);
p2 = p(5:8);

q1 = q(1:4);
q2 = q(5:8);

x = [ mulpq( p1, q1 );  mulpq( p1, q2 )  + mulpq( p2, q1 )  ];
