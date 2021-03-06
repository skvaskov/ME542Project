%generate polynomial that returns positive when we are on the track
clear all
close all
clc
load('CircuitOfAmerica.mat')
figure(1)
plot(Track.bl(1,:),Track.bl(2,:),'r')
hold on
plot(Track.br(1,:),Track.br(2,:),'r')
plot(Track.br(1,292),Track.br(2,292),'k*')
plot(Track.bl(1,292),Track.bl(2,292),'k*')
plot(Track.br(1,370),Track.br(2,370),'k*')
plot(Track.bl(1,370),Track.bl(2,370),'k*')

%phase 8 indexes are 292-370
start=290;
fin=372;
%find approximation for left bound
xl=Track.bl(1,start:fin);
yl=Track.bl(2,start:fin);
pl=polyfit(xl,yl,2);
ylF=polyval(pl,linspace(xl(1),xl(end)));
xr=Track.br(1,start:fin);
yr=Track.br(2,start:fin);
pr=polyfit(xr,yr,2);
yrF=polyval(pr,linspace(xr(1),xr(end)));
figure(1)
hold on
plot(linspace(xl(1),xl(end)),ylF)
plot(linspace(xr(1),xr(end)),yrF)

%get centerline
yc=Track.cline(2,start:fin);
xc=Track.cline(1,start:fin);

rightcheck=yc-polyval(pr,xc);
leftcheck=yc-polyval(pl,xc);
figure(2)
plot(leftcheck)
hold on
plot(rightcheck)
legend('left','right')
xbl=[xl(end);yl(end)];
xbr=[xr(end);yr(end)];
Xf=[xc(end);yc(end)];
cd=yc-xbr(2)-(xbl(2)-xbr(2))/(xbl(1)-xbr(1))*(xc-xbr(1));