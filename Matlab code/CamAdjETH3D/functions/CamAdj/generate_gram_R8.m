%% Generate Gram matrix for two-view triangulation
clc;
clear all;
close all;
addpath(genpath('./functions'));
% define parameters and variables
% 2D point in camera coordinate (inv(K)*[u;v;1])
syms ux uy;    % 
syms upx upy;  % 
syms cx cy cz; % rotation in cayley
syms rx ry rz; % ranslation vector
syms gamma gamma_prime;
u = [ux; uy; 1];
up = [upx; upy; 1];
c = [cx; cy; cz];
r = [rx; ry; rz];
I = eye(3);
c_cross = skew(c);
% polynomials
f = gamma_prime.*(I - c_cross)*up - gamma.*(I + c_cross)*u + r;
f
% compute gram matrix Qf_i
y = [c;r;gamma;gamma_prime;1];
G = sym('g', [length(y) length(y)]);
G = triu(G,1) + triu(G,0).' ;
F = y.'*G*y;
F = collect(F,[cx cy cz rx ry rz gamma gamma_prime]);

% each 3D-2D correspondence generates 3 polynomials  
polynomials_id = 3;
f_i = f(polynomials_id);
f_i = collect(f_i,[cx cy cz rx ry rz gamma gamma_prime]);
% find out each element of Qf
Fp = simplify(F - f_i); % F - f_i = 0
Fp = collect(Fp,[cx cy cz rx ry rz gamma gamma_prime]);
Fc = coeffs(Fp,[cx cy cz rx ry rz gamma gamma_prime]);
Fc = fliplr(Fc);
Fc = Fc.';
eqs = Fc == 0;
disp('===================================================================')
disp('the results show the equations of each element in gram matrix G')
-eqs
