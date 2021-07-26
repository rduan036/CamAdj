function plot_pose( pose, color  )
	
O = pose(1:3,4);
x = pose(1:3,1);
y = pose(1:3,2);
z = pose(1:3,3);

d = 0.1;
px = O + d*x;
py = O + d*y;
pz = O + d*z;

pxx = O + 1.5*d*x;
pyy = O + 1.5*d*y;
pzz = O + 1.5*d*z;

hold on;
plot3( [O(1), px(1) ], [O(2), px(2) ],  [O(3), px(3) ], color, 'LineWidth', 2 );   text( pxx(1), pxx(2), pxx(3), 'X' );
plot3( [O(1), py(1) ], [O(2), py(2) ],  [O(3), py(3) ], color, 'LineWidth', 2 );   text( pyy(1), pyy(2), pyy(3), 'Y' );
plot3( [O(1), pz(1) ], [O(2), pz(2) ],  [O(3), pz(3) ], color, 'LineWidth', 2 );    text( pzz(1), pzz(2), pzz(3), 'Z' );



