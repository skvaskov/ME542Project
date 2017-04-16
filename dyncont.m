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
