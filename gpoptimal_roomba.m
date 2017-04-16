%get to a final point in the minimum time possible obeying path constraints
%for unicycle model and only 1 phase now

%-------------------------------------------------------------------------%
%                          Initial Setup                                  %
%-------------------------------------------------------------------------%
close all
clear all
clc
load('F1CarData.mat')
load('CircuitOfAmerica.mat')

%specify start and end index of phase
start=11;
fin=21;
%very conservative guess for final time
t0=0;
T=(Track.arc_s(fin)-Track.arc_s(start))/10;
t=linspace(t0,T,1000);
dt=t(2)-t(1);
xc0=[Track.cline(1:2,start)];%Track.theta(start)]; initial centerline point
xcF=[Track.cline(1:2,fin)];%Track.theta(fin)]; final centerline point
%boundary positions of finish
xlF=Track.bl(1:2,fin);
xrF=Track.br(1:2,fin);


%-------------------------------------------------------------------------%
%----------------- Provide All Bounds for Problem ------------------------%
%-------------------------------------------------------------------------%
iphase=1;
bounds.phase(iphase).initialtime.lower = t0;
bounds.phase(iphase).initialtime.upper = t0;
bounds.phase(iphase).finaltime.lower = 0;
bounds.phase(iphase).finaltime.upper = T;
bounds.phase(iphase).initialstate.lower = xc0';
bounds.phase(iphase).initialstate.upper = xc0';
bounds.phase(iphase).state.lower = [-400 -300]; 
bounds.phase(iphase).state.upper = [1600 1000]; 
bounds.phase(iphase).finalstate.lower =  [-400 -300];
bounds.phase(iphase).finalstate.upper = [1600 1000]; 
bounds.phase(iphase).control.lower = [-100 -100];
bounds.phase(iphase).control.upper = [100 100];
%cost
 bounds.phase(iphase).integral.lower=0;
 bounds.phase(iphase).integral.upper=1e6;
%path constraints
bounds.phase(iphase).path.lower=[0 -1e6 0];
bounds.phase(iphase).path.upper=[85 0 1e6];
%terminate trajectory at final state
bounds.eventgroup(iphase).lower=-1e6;
bounds.eventgroup(iphase).upper=0;

%-------------------------------------------------------------------------%
%                          Initial Guess                                  %
%-------------------------------------------------------------------------%
guess.phase(iphase).time     = [ t0; T ];
guess.phase(iphase).state    = [ xc0'; xcF'];


guess.phase(iphase).control  = [7.4,-6;7.4,-6 ];
guess.phase(iphase).integral = 23;

%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
mesh.method          = 'hp-LiuRao-Legendre';
mesh.tolerance       = 1e-7;
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
ylabel('k_x')
subplot(3,1,3)
plot(gpops_t,u_ol(:,2))
xlabel('time')
ylabel('k_y')
%-------------------------------------------------------------------------%
%                          Functions                                      %
%-------------------------------------------------------------------------%
function phaseout = dyncont(input)
t=input.phase.time;
x=input.phase.state(:,1);
y=input.phase.state(:,2);
xcF=input.auxdata.xcF;
xbl=input.auxdata.xblF;
xbr=input.auxdata.xbrF;
k1=input.phase.control(:,1);
k2=input.phase.control(:,2);
dx=k1;
dy=k2;
phaseout.dynamics=[dx,dy];
%left=-.1990*x-.02554*y-.1355;
%right=.1722*x+.2188*y+4.1726;
right=y-144.503901491793+0.788349918916939*(x+207.011572367274);
left=y-109.677627119228+0.782251827221814*(x+138.588713481620);
%right line y=-.7872x
phaseout.integrand=sqrt((x-ones(size(x))*xcF(1)).^2+(y-ones(size(y))*xcF(2)).^2);
phaseout.path=[sqrt(dx.^2+dy.^2) left right];
end

function output=endpoint(input)
Xf=input.phase(1).finalstate;
T=input.phase(1).finaltime;
xbl=input.auxdata.xblF;
xbr=input.auxdata.xbrF;
%end of phase 1 is determined by the crossing the line between the two
%boundary points
output.eventgroup(1).event=Xf(2)-xbr(2)-(xbl(2)-xbr(2))/(xbl(1)-xbr(1))*(Xf(1)-xbr(1));
% objective is to minimize final time
output.objective=input.phase(1).integral+T;
end