function [DIST,ximp,yimp,GrandObstacle]=USPATCH_act(Portee,angles,Obstacles,x,y,theta,Bruitage,CoefTexture_Wall)

Sobst=size(Obstacles);
if isfield(Obstacles,'Dist_Detect')
    GrandObstacle=zeros(Sobst(2)*3,3);
else
    GrandObstacle=zeros(Sobst(2)*3,2);
end
for i=1:Sobst(2)
    if isfield(Obstacles,'Dist_Detect')
        if Obstacles(i).Dist_Detect>=0 || CoefTexture_Wall == 0
            GrandObstacle(i:i+1,:) = [Obstacles(i).Pos_vertex(:,1)  Obstacles(i).Pos_vertex(:,2) [i;i]];
            GrandObstacle(i+2,:) = [NaN NaN NaN];
        else
%            nondetectable=1;
        end
    else
        GrandObstacle(i:i+1,:) = [Obstacles(i).Pos_vertex(:,1)  Obstacles(i).Pos_vertex(:,2)];
        GrandObstacle(i+2,:) = [NaN NaN];
    end
end

%transforme un angle h compris entre 0 et 2pi en un angle g compris entre -pi et pi

h=angles+theta;

if h<-pi
    h = h+2*pi;
elseif h>pi
    h = h-2*pi;
end

x1=x+Portee*cos(h);
y1=y+Portee*sin(h);

DIST=Portee*ones(length(angles),1);

Bruit=Bruitage;

ximp = zeros(1,length(x1));
yimp = zeros(1,length(x1));

for kk=1:length(x1)
    [uu, vv, ii]= polyxpoly([x x1(kk)],[y y1(kk)],GrandObstacle(:,1),GrandObstacle(:,2));
    [a, b]=min((uu-x).^2+(vv-y).^2);
    
    if ~isempty(a)
        if isfield(Obstacles,'Dist_Detect') &&  CoefTexture_Wall ~= 0
            if sqrt(a)<=(Obstacles(GrandObstacle(ii(b,2),3)).Dist_Detect*CoefTexture_Wall)%Obstacles(ii(b)).Dist_Detect
                [ Xn, Yn, Dn ]=Add_mesurement_noise(x, y, uu(b), vv(b), Portee, Bruit);               
                DIST(kk)=Dn;%sqrt(a);
                ximp(kk)=Xn;%uu(b);
                yimp(kk)=Yn;%vv(b);
            else
                ximp(kk)=x1(kk);
                yimp(kk)=y1(kk);
            end
        else
            DIST(kk)=sqrt(a);
            ximp(kk)=uu(b);
            yimp(kk)=vv(b);
        end
    else
        ximp(kk)=x1(kk);
        yimp(kk)=y1(kk);
    end
    
end