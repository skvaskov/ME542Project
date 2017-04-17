function [gamma] = GBrute(t)
%UNTITLED2 Summary of this function goes here
%   OL is open loop control in the form of t;gamma
OL=[-1e-6,0,4,     8,  10, 11,13, 15,    17,    19,   20, 21,  23.75, 25,26,27.25,  31,   33,34,34.05,35,40;...
    0,    0,-.005,.325,0,-.01,0,-.0125,-.165,-.1,   0, 0,  -.1, .15 ,0,  0  .1067,0.1,0.085,0.075,0,0];
gamma=interp1(OL(1,:),OL(2,:),t);
end

