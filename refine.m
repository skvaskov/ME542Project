function [refined] = refine(cline,nuber)
%import cline in 2XN matrix. number is the amount of points we want to end
%up with for now make nuber an even multiple of the number of points
origlength=size(cline,2);
frac=nuber/origlength;
x1=linspace(cline(1,1),cline(1,2),frac);
y1=interp1([cline(1,1),cline(1,2)],[cline(2,1),cline(2,2)],x1);
refined=[x1;y1];
for i=2:origlength-1
    x=linspace(cline(1,i),cline(1,i+1),frac+1);
    x=x(2:end);
    y=interp1([cline(1,i),cline(1,i+1)],[cline(2,i),cline(2,i+1)],x);
    ADD=[x;y];
    refined=[refined,ADD];

end

