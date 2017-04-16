clear all
clc
syms m l gamma d sigma m0 R k gdot x y psi real
model=[m/l*tan(gamma)*(1-d/l)*sigma^2-(m0-m*d/l)/(m+m0*tan(gamma)^2)*((R-k*sigma^2)*tan(gamma)+m*gdot*sigma/(cos(gamma)^2));...
    1/cos(gamma)*(m*d/l^2*sigma^2*tan(gamma)+m0/(m+m0*tan(gamma)^2)*((R-k*sigma^2)*tan(gamma)+m*gdot*sigma/cos(gamma)^2))];

matlabFunction(model,'file','tireforce',...
     'vars',[m,m0,l,d,k,x,y,psi,sigma,gamma,R,gdot]);