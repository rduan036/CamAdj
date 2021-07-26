function Gn = getGn(varMinMax)
% Given the bounds on variables, generate matrix for second degree poly
nVar = size(varMinMax,1);
Gn = zeros(nVar+1,nVar+1,nVar+1);
N = sum(max((varMinMax').^2));
for idx = 1:nVar
    Qtemp = zeros(nVar+1,nVar+1);
    l = varMinMax(idx,1); u = varMinMax(idx,2);
    
    Qtemp(idx,idx) = -1;
    Qtemp(idx,nVar+1) = (l+u)/2;  Qtemp(nVar+1,idx) = (l+u)/2;
    Qtemp(nVar+1,nVar+1) = -l*u;
    Gn(:,:,idx) = Qtemp;
end
Gn(:,:,end) = diag(-ones(nVar+1,1));
Gn(end) = N;
end