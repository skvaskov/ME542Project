function [R] = RBrute5(t)
%UNTITLED2 Summary of this function goes here
%   OL is open loop control in the form of t;R
OL=[0,   .05,       1  ,1.05,    3,4.5,4.505,   5, 8.75,9.25,9.5,  10.05 , 11.5,12,14,15,25;...
    5500,-10000,-10000, -10000,  0,0,  5500,  5500, 5500,5500,0,-10000,-10000,0,  3500,5500,5500];
R=interp1(OL(1,:),OL(2,:),t);

end

