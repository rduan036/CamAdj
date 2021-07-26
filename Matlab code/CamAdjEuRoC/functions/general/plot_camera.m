function plot_camera( pose_cam, color )

p1 = [  0.05;  0.05; 0 ];
p2 = [  0.05; -0.05; 0 ];
p3 = [ -0.05; -0.05; 0 ];
p4 = [ -0.05;  0.05; 0 ];


L = 0.05;
z = [0; 0; 1];

p5 = p1 + L*z;
p6 = p2 + L*z;
p7 = p3 + L*z;
p8 = p4 + L*z;

a = L*z;
b = [  0.025;  0.025; 0 ] + 1.2*a;
c = [  0.025; -0.025; 0 ] + 1.2*a;
d = [ -0.025; -0.025; 0 ] + 1.2*a;
e = [ -0.025;  0.025; 0 ] + 1.2*a;

%% move camera to a given pose
cam_points = pose_cam*[ p1, p2, p3, p4, p5, p6, p7, p8, -a, -b, -c, -d, -e; ones(1,13)];
p1 = cam_points(1:3,1);  
p2 = cam_points(1:3,2);
p3 = cam_points(1:3,3);
p4 = cam_points(1:3,4);

p5 = cam_points(1:3,5);
p6 = cam_points(1:3,6);
p7 = cam_points(1:3,7);
p8 = cam_points(1:3,8);

a = cam_points(1:3,9);
b = cam_points(1:3,10);
c = cam_points(1:3,11);
d = cam_points(1:3,12);
e = cam_points(1:3,13);


%% plot camera
plot_plane( p1, p2, p3, p4, color );
plot_plane( p5, p6, p7, p8, color );
plot_plane( p1, p5, p8, p4, color );
plot_plane( p2, p3, p7, p6, color );

plot_triangle( a, b, c, color );
plot_triangle( a, d, e, color );
plot_plane( b, c, d, e, color );






