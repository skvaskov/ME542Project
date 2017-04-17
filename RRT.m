%TRY AND USE RRT TO GET FEASIBLE TRAJECTORY
clear all
close all
clc
load('CircuitOfAmerica.mat')
load('F1CarData.mat')
%specify start index and final time and timestep size
start=362;
Tf=1;
dt=0.05;
t=0:dt:Tf;
v0=40;
x0=[Track.cline(1:2,start);Track.theta(start);v0];

%specify Grids for Force and steering input
R=-10000:5000:5000;
G=-.5:.1:.5;
[Rgrid,Ggrid,Z]=meshgrid(R,G,t);