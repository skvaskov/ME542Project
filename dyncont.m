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
phaseout.integrand=sqrt((x1(:,1)-ones(size(x1(:,1)))*xcF(1)).^2+(x1(:,2)-ones(size(x1(:,2)))*xcF(2)).^2);
phaseout.path=[left right Ftire];
end
