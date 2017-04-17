function [gamma] = GBrute(t)
%UNTITLED2 Summary of this function goes here
%   OL is open loop control in the form of t;gamma
OL=[-1e-6,0,4,8,10,13,15,17,19,23;...
    0,0,0,.35,0,0,-.01,-.125,-.125,0];
gamma=interp1(OL(1,:),OL(2,:),t);
end

