function R = cayley_c2R(c)
    I = eye(3);
    skew_c = skew(c);
    R = pinv(I - skew_c) * (I + skew_c);
return