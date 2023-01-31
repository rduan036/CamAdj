function L = image_jacobian(reprojectpoints, depths)

x = reprojectpoints(1,:);
y = reprojectpoints(2,:);

N = length(x);

L = [];

for ii = 1:N
    L = [L; compute_jacobian(x(ii),y(ii),depths(ii))];
end

return