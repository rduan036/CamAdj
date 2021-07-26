function plot_triangle( p1, p2, p3, color )

x = [p1(1); p2(1); p3(1); p1(1) ];
y = [p1(2); p2(2); p3(2); p1(2) ];
z = [p1(3); p2(3); p3(3); p1(3) ];

plot3( x, y, z, color );