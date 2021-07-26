function Lx = compute_jacobian(x,y,Z)

Lx = [-1/Z, 0, x/Z, x*y, -(1+x^2), y;
    0, -1/Z, y/Z, 1+y^2, -x*y, -x];

end