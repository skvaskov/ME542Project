function [gamma] = GBrute5(t)
%UNTITLED2 Summary of this function goes here
%   OL is open loop control in the form of t;gamma
OL=[-.05,   0,   .05,2,  3, 4,  4.5, 5,8,10.75,11.5, 12,12.5,13,15,15.65,17,17.25,17.5,25;...
    -5e-4,-2.5e-4,0, 0, .1,0.115,0.1,0,-.001,0,.1,.125,.1,.1,0.05,0,0,-.0025,0,0];
gamma=interp1(OL(1,:),OL(2,:),t);
end

