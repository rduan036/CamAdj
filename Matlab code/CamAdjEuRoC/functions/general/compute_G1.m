function G = compute_G1(Xx,Xy,Xz,xx,xy)

q1_1 = Xy - Xz*xy;
q1_2 = (Xz*xx - Xx)/2;
q1_3 = (Xx*xy - Xy*xx)/2;
q1_4 = 0;
q1_5 = xy/2;
q1_6 = 1/2;
q1_7 = (2*Xz + 2*Xy*xy)/2;
q2_2 = 0;
q2_3 = 0;
q2_4 = 0;
q2_5 = (- xx)/2;
q2_6 = 0;
q2_7 = (- Xx*xy - Xy*xx)/2;
q3_3 = 0;
q3_4 = 0;
q3_5 = 0;
q3_6 = (- xx)/2;
q3_7 = (- Xx - Xz*xx)/2;
q4_4 = 0;
q4_5 = 0;
q4_6 = 0;
q4_7 = 0;
q5_5 = 0;
q5_6 = 0;
q5_7 =  -1/2;
q6_6 = 0;
q6_7 = xy/2;
q7_7 = Xz*xy - Xy;

G = [q1_1, q1_2, q1_3, q1_4, q1_5, q1_6, q1_7;
    q1_2, q2_2, q2_3, q2_4, q2_5, q2_6, q2_7;
    q1_3, q2_3, q3_3, q3_4, q3_5, q3_6, q3_7;
    q1_4, q2_4, q3_4, q4_4, q4_5, q4_6, q4_7;
    q1_5, q2_5, q3_5, q4_5, q5_5, q5_6, q5_7;
    q1_6, q2_6, q3_6, q4_6, q5_6, q6_6, q6_7;
    q1_7, q2_7, q3_7, q4_7, q5_7, q6_7, q7_7];


end