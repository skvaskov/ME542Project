clear all
close all
clc
load('F1CarData.mat')
load('openloopcontrolbuff.mat')
m=CarParameter.m;
m0=CarParameter.m0;
l=CarParameter.w;
d=CarParameter.b;
k=CarParameter.k;
F1=zeros(length(gpops_t1),2);
F2=zeros(length(gpops_t2),2);

for i=1:length(gpops_t1)
    F1(i,:)=tireforce(m,m0,l,d,k,gpops_states1(i,1),gpops_states1(i,2),...
        gpops_states1(i,3),gpops_states1(i,4),gpops_states1(i,5),u_ol1(i,1),u_ol1(i,2));
    
end

for i=1:length(gpops_t2)
    F2(i,:)=tireforce(m,m0,l,d,k,gpops_states2(i,1),gpops_states2(i,2),...
        gpops_states2(i,3),gpops_states2(i,4),gpops_states2(i,5),u_ol2(i,1),u_ol2(i,2));
end
left=0.017498445482765*gpops_states2(:,1).^2+0.00990885287913384*gpops_states2(:,2).^2+0.0318848049847519*gpops_states2(:,1).*gpops_states2(:,2)-3.37853614766604*gpops_states2(:,1)-5.21831613500128*gpops_states2(:,2)-79.8843013598673;
 right=-0.00771863579990068*gpops_states2(:,1).^2-0.004529479645282*gpops_states2(:,2).^2-0.0107264263505469*gpops_states2(:,1).*gpops_states2(:,2)+1.90051932193427*gpops_states2(:,1)+1.39418286431919*gpops_states2(:,2)-63.1759701263474;
subplot(2,1,1)
plot(F1(:,1))
hold on
plot(F1(:,2))
subplot(2,1,2)
plot(F2(:,1))
hold on
plot(F2(:,2))
figure
plot(left)
hold on
plot(right)
legend('left','right')