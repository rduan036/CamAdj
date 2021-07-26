function x = uthetat2dq( u, theta, t )
% theta : rotation around an axis
% u : rotation axis
% t : translation 
		
q_rot = [ cos( theta/2 ); sin( theta/2 )*u   ];

q_tr = 0.5*mulpq( [0; t], q_rot );

x = [ q_rot; q_tr ];
