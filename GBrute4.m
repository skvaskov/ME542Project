function [gamma] = GBrute4(t)
%UNTITLED2 Summary of this function goes here
%   OL is open loop control in the form of t;gamma
OL=[-1e-6,0,   2.5,3,3.5, 6, 8.5,9 ,  10.5,11,12,13,14,   15,  16,   17,      18,   20;...
    0,    0,-.01,  0,0,  -.1, 0,0,  -.05,-.05,0,0 ,-.05,-.025,-.025, -.025,   -.01, 0];
gamma=interp1(OL(1,:),OL(2,:),t);
end

