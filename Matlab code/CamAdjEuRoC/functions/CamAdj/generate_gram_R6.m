%% Generate G
clc;
clear all;
close all;
% define parameters and variables
syms Xx Xy Xz; % 3D point in world coordinate
syms xx xy;    % 2D point in camera coordinate (inv(K)*[u;v;1])
syms cx cy cz; % rotation in cayley
syms tx ty tz; % ranslation vector
X = [Xx; Xy; Xz];
x = [xx; xy; 1];
c = [cx; cy; cz];
t = [tx; ty; tz];
I = eye(3);
c_cross = skew(c);
% polynomials
f = skew((I - c_cross)*x)*((I + c_cross)*X + t);
% each 3D-2D correspondence generates 3 polynomials  
polynomials_id = 1;
f = f(polynomials_id);
f = collect(f,[cx cy cz tx ty tz]);
% compute gram matrix Qf_i
V = [c;t;1];
G = sym('q', [length(V) length(V)]);
G = triu(G,1) + triu(G,0).' ;
F = V.'*G*V;
F = collect(F,[cx cy cz tx ty tz]);

g = simplify(F - f);
g = collect(g,[cx cy cz tx ty tz]);
c = coeffs(g,[cx cy cz tx ty tz]);
c = fliplr(c);
c = c.';
eqs = c == 0;
disp('===================================================================')
disp('the results show the equations of each element in gram matrix Qf')
-eqs
