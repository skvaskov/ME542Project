function [R] = RBrute(t)
%UNTITLED2 Summary of this function goes here
%   OL is open loop control in the form of t;R
OL=[0,    0.05,   2.3,    3, 4.5,8.5, 9,   13, 14,   15,17,18,20,21,22, 22.4,  22.25, 23, 23.5,  24,24.1, 26, 27,   28.25,30.5,30.75,31,31.9,32,32.25,35,35.1,40;...
    -10000,5200,-10000,-7500,0,  0, 5500, 5500,0 ,-10000,0,0,4500,0,5500, 0, -10000,-5000, 3500, 0, -10000,3500, 5500,750,0, -4000 ,0, 0,-10000,0,0,5500,5500 ];
R=interp1(OL(1,:),OL(2,:),t);

end

