function [R] = RBrute(t)
%UNTITLED2 Summary of this function goes here
%   OL is open loop control in the form of t;R
OL=[0,2,3,4.5,8.5,9,13,14,15,17,18,20,23;...
    5500,-10000,-7500,0,0,5500,5500,0,-10000,0,0,4500,3500];
R=interp1(OL(1,:),OL(2,:),t);

end

