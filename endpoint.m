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

output.objective=input.phase(1).integral+1000*(T+(Xf(4)-40)^2+10*(Xf(3)-xcF(3))^2);
end
