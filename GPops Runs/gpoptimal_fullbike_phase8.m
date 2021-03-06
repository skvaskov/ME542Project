%get to a final point in the minimum time possible obeying path constraints
%for unicycle model and only 1 phase now this is for the long straightaway

%-------------------------------------------------------------------------%
%                          Initial Setup                                  %
%-------------------------------------------------------------------------%
close all
clear all
clc
%get initial condition
load('mattraj.mat')
x0=[y2(end,:) u2(end,10)];
t0=t2(end);
clearvars -except x0 t0
load('F1CarData.mat')
load('CircuitOfAmerica.mat')

%specify start and end index of phase
start=292;
fin=362;
%very conservative guess for final time
T=(Track.arc_s(fin)-Track.arc_s(start))/10+t0;
t=linspace(t0,T,1000);
dt=t(2)-t(1);
xc0=[Track.cline(1:2,start);Track.theta(start)-2*pi]; %initial centerline point
xcF=[Track.cline(1:2,fin);Track.theta(fin)-2*pi]; %final centerline point
%boundary positions of finish
xlF=Track.bl(1:2,fin);
xrF=Track.br(1:2,fin);
param=[CarParameter.m CarParameter.m0 CarParameter.w CarParameter.b CarParameter.k];
%-------------------------------------------------------------------------%
%----------------- Provide All Bounds for Problem ------------------------%
%-------------------------------------------------------------------------%
iphase=1;
bounds.phase(iphase).initialtime.lower = t0;
bounds.phase(iphase).initialtime.upper = t0;
bounds.phase(iphase).finaltime.lower = 0;
bounds.phase(iphase).finaltime.upper = T;
bounds.phase(iphase).initialstate.lower = [xc0' 30 0];
bounds.phase(iphase).initialstate.upper = [xc0' 30 0];
 bounds.phase(iphase).state.lower = [-400 -300 -pi 0 -.5]; 
 bounds.phase(iphase).state.upper = [1600 1000 pi 150 .5]; 
 bounds.phase(iphase).finalstate.lower =  [-400 -300 -pi 0 -.5];
 bounds.phase(iphase).finalstate.upper = [1600 1000 pi 150 .5]; 

bounds.phase(iphase).control.lower = [-10000 -.5];
bounds.phase(iphase).control.upper = [5500 .5];
%cost
  bounds.phase(iphase).integral.lower=0;
  bounds.phase(iphase).integral.upper=10000;
%path constraints
bounds.phase(iphase).path.lower=[2 -100 -2500 -2500];
bounds.phase(iphase).path.upper=[100 2 2500 2500];
%terminate trajectory at cross of "finish line"
bounds.eventgroup(iphase).lower=-1e6;
bounds.eventgroup(iphase).upper=0;

%-------------------------------------------------------------------------%
%                          Initial Guess                                  %
%-------------------------------------------------------------------------%
guess.phase(iphase).time     = [ t0; T ];
guess.phase(iphase).state    = [ x0; xcF' 40 0];


guess.phase(iphase).control  = [-10000,0;-10000,0 ];
guess.phase(iphase).integral = 0;

%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
mesh.method          = 'hp-LiuRao-Legendre';
mesh.tolerance       = 1e-3;
mesh.phase(1).colpoints = 4 * ones(1,10);
mesh.phase(1).fraction = 0.1 * ones(1,10);

%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%        
%-------------------------------------------------------------------------%
setup.name                           = 'optimalControl';
setup.functions.continuous           = @dyncont;
setup.functions.endpoint             = @endpoint;
setup.displaylevel                   = 1;
setup.bounds                         = bounds;
setup.guess                          = guess;
setup.mesh                           = mesh;
setup.nlp.solver                     = 'ipopt';
setup.nlp.snoptoptions.tolerance     = 1e-4;
setup.nlp.snoptoptions.maxiterations = 20000;
setup.nlp.ipoptoptions.linear_solver = 'ma57';
setup.nlp.ipoptoptions.tolerance     = 1e-4;
% setup.derivatives.supplier           = 'adigator';
% setup.derivatives.derivativelevel    = 'second';
setup.method                         = 'RPM-Differentiation';

%-------------------------------------------------------------------------%
% ---------------------Assemble Auxiliary Data --------------------------%        
%-------------------------------------------------------------------------%
setup.auxdata.xcF=xcF;
setup.auxdata.xblF=xlF;
setup.auxdata.xbrF=xrF;
setup.auxdata.cardata=param;
%-------------------------------------------------------------------------%
%---------------------- Solve Problem Using GPOPS2 -----------------------%
%-------------------------------------------------------------------------%
tic
output = gpops2(setup);
toc

u_ol = output.result.solution.phase.control;
gpops_t = output.result.solution.phase.time;
gpops_states=output.result.solution.phase.state;
%NOTE: your gpops time vector will be a different length than the time
%vector for your simulation. That's why I also extracted the time vector in
%gpops so you can plot it. If you want to make the u_ol vector the same
%length as your original time vector, use this:
% u_ol = spline(gpops_t, u_ol');
% u_ol = ppval(u_ol, t(:));

figure
subplot(3,1,1)
plot(gpops_states(:,1),gpops_states(:,2))
hold on
plot(xc0(1),xc0(2),'k*')
plot(xcF(1),xcF(2),'k*')
plot(Track.bl(1,start:fin),Track.bl(2,start:fin),'r')
plot(Track.br(1,start:fin),Track.br(2,start:fin),'r')
xlabel('X')
ylabel('Y')
subplot(3,1,2)
plot(gpops_t,u_ol(:,1))
xlabel('time')
ylabel('R')
subplot(3,1,3)
plot(gpops_t,u_ol(:,2))
xlabel('time')
ylabel('\gamma`')
%-------------------------------------------------------------------------%
%                          Functions                                      %
%-------------------------------------------------------------------------%
function phaseout = dyncont(input)
t=input.phase.time;
% x=input.phase.state(:,1);
% y=input.phase.state(:,2);
% psi=input.phase.state(:,3);
% sigma=input.phase.state(:,4);
% gamma=input.phase.state(:,5);
x1=input.phase.state;
u=input.phase.control;
xcF=input.auxdata.xcF;
% R=input.phase.control(:,1);
% gdot=input.phase.control(:,2);
dx=zeros(length(t),size(x1,2));
Ftire=zeros(length(t),2);
   P = input.auxdata.cardata;
    P = num2cell(P);
for i=1:length(t)
    X=num2cell(x1(i,:));
    U = num2cell(u(i,:));
    dx(i,:) = gpopsdyn(P{:},X{:},U{:});
    Ftire(i,:)=tireforce(P{:},X{:},U{:});
end
phaseout.dynamics=dx;
right=x1(:,2)-0.0001247564164*x1(:,1).^2+0.0221132121353*x1(:,1)-612.8982167785507;
left=x1(:,2) -0.0001144350496*x1(:,1).^2+0.0021560732820*x1(:,1)-589.3732440353729;
phaseout.integrand=sqrt((x1(:,1)-ones(size(x1(:,1)))*xcF(1)).^2+(x1(:,2)-ones(size(x1(:,2)))*xcF(2)).^2+...
    (x1(:,3)-ones(size(x1(:,3)))*xcF(3)).^2+(x1(:,3)-ones(size(x1(:,3)))*40).^2);
phaseout.path=[left right Ftire];
end

function output=endpoint(input)
Xf=input.phase(1).finalstate;
T=input.phase(1).finaltime;
xbl=input.auxdata.xblF;
xbr=input.auxdata.xbrF;
xcF=input.auxdata.xcF;
%end of phase 1 is determined by the crossing the line between the two
%boundary points
output.eventgroup(1).event=Xf(2)-xbr(2)-(xbl(2)-xbr(2))/(xbl(1)-xbr(1))*(Xf(1)-xbr(1));
% objective is to minimize final time
%compute total distance away from endpoint

output.objective=input.phase(1).integral+T;
end