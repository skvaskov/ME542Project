%get to a final point in the minimum time possible obeying path constraints
%for unicycle model and only 1 phase now

%-------------------------------------------------------------------------%
%                          Initial Setup                                  %
%-------------------------------------------------------------------------%
close all
clear all
clc

%get initial condition and time from previous phase
load('openloopcontrolbuff.mat')
x0=gpops_states2(end,:);
t0=gpops_t2(end);
clearvars -except x0 t0
%
load('F1CarData.mat')
load('CircuitOfAmerica.mat')
%specify start and end index of phase
start=36;
fin=45;
%very conservative guess for final time
T=t0+(Track.arc_s(fin)-Track.arc_s(start))/10+25;
t=linspace(t0,T,1000);
dt=t(2)-t(1);
xc0=[Track.cline(1:2,start);Track.theta(start)]; %initial centerline point
xcF=[Track.cline(1:2,fin);Track.theta(fin)]; %final centerline point
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
bounds.phase(iphase).initialstate.lower = x0;
bounds.phase(iphase).initialstate.upper = x0;
bounds.phase(iphase).state.lower = [-400 -300 -pi 0 -.5]; 
bounds.phase(iphase).state.upper = [1600 1000 pi 150 .5]; 
bounds.phase(iphase).finalstate.lower =  [-400 -300 -pi 0 -.5];
bounds.phase(iphase).finalstate.upper = [1600 1000 pi 150 .5]; 

bounds.phase(iphase).control.lower = [-10000 -.25];
bounds.phase(iphase).control.upper = [5500 .25];
%cost
  bounds.phase(iphase).integral.lower=0;
  bounds.phase(iphase).integral.upper=10000;
%path constraints
bounds.phase(iphase).path.lower=[0 0 -5000 -4500];
bounds.phase(iphase).path.upper=[1e6 1e6 5000 4500];
%terminate trajectory at cross of "finish line"
bounds.eventgroup(iphase).lower=0;
bounds.eventgroup(iphase).upper=1e6;

%-------------------------------------------------------------------------%
%                          Initial Guess                                  %
%-------------------------------------------------------------------------%
 guess.phase(iphase).time     = [ t0; T ];
guess.phase(iphase).state    = [ x0; xcF' 0 0];
% guess.phase(iphase).time=linspace(t0,T,fin-start+1)';
% clinestates=[Track.cline(1:2,start:fin);Track.theta(start:fin);...
%     10*ones(1,fin-start+1);zeros(1,fin-start+1)];
% guess.phase(iphase).state=[x0;clinestates(:,2:end)'];
% contr=[1000*ones(fin-start+1,1) zeros(fin-start+1,1)];
% guess.phase(iphase).control  = contr;
guess.phase(iphase).control = [5000 0;0 0];
guess.phase(iphase).integral = 1000;

%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
mesh.method          = 'hp-LiuRao-Legendre';
mesh.tolerance       = 1e-6;
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
xbl=input.auxdata.xblF;
xbr=input.auxdata.xbrF;
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
left=0.00584113818737157*x1(:,1).^2+0.000715776610793797*x1(:,2).^2+0.00410085041108458*x1(:,1).*x1(:,2)-2.0486203451185*x1(:,1)-0.733114404223798*x1(:,2)+167.248959747038;
right=-0.00387663805047578*x1(:,1).^2-0.00150289791070212*x1(:,2).^2-0.00547027011226241*x1(:,1).*x1(:,2)+1.06576225836893*x1(:,1)+1.00011236472944*x1(:,2)-39.2651317560038;

%phaseout.integrand=sqrt((x1(:,1)-ones(size(x1(:,1)))*xcF(1)).^2+(x1(:,2)-ones(size(x1(:,2)))*xcF(2)).^2);
phaseout.integrand=x1(:,2)-xbr(2)-(xbl(2)-xbr(2))/(xbl(1)-xbr(1))*(x1(:,1)-xbr(1));
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

output.objective=input.phase(1).integral+1000*T+100*(Xf(3)-xcF(3))^2;
end