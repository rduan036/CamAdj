function c = cayley_R2c(R)
    I = eye(3);
    skew_c = (R - I)*pinv(R + I);
    c = [skew_c(3,2); skew_c(1,3); skew_c(2,1)];
return