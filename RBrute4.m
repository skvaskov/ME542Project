function [R] = RBrute4(t)
%UNTITLED2 Summary of this function goes here
%   OL is open loop control in the form of t;R
OL=[0,    2.5,  2.6,   4,      5, 6,  8,  10,10.5,  10.75,12,12.1,12.75, 13, 13.25,15,15.1,20;...
    5500, 5500,-10000,-10000, 0, 2500,5500, 0,-5000,0,   0,  5500,5500,-10000,0,  0,5500,5500];
R=interp1(OL(1,:),OL(2,:),t);

end

