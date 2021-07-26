function G = compute_G(Xx,Xy,Xz,xx,xy)

G = [];
G1 = compute_G1(Xx,Xy,Xz,xx,xy);
G2 = compute_G2(Xx,Xy,Xz,xx,xy);
G3 = compute_G3(Xx,Xy,Xz,xx,xy);
G(:,:,1) = G1;
G(:,:,2) = G2;
G(:,:,3) = G3;

end