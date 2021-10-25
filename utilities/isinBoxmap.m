function [ res ] = isinBoxmap( x,y,Obstacles )
%ISINBOXMAP Return if the coordonates x,y,given are within in  the map described by the given obstacles.
%   Return 0 when false
res=0;


Sobst=size(Obstacles);
GrandObstacle=[];
for i=1:Sobst(2)
    GrandObstacle=[GrandObstacle;Obstacles(i).Pos_vertex(:,1)  Obstacles(i).Pos_vertex(:,2)];
    GrandObstacle=[GrandObstacle; NaN NaN];
end

if sum(GrandObstacle(:, 1) == x & GrandObstacle(:, 2) == y)==0

    h=(rand(7,1)*2*pi)-pi;

    x1=x+99999*cos(h);
    y1=y+99999*sin(h);

    Sucess=0;
    for kk=1:length(x1)
        [uu vv]= polyxpoly([x x1(kk)],[y y1(kk)],GrandObstacle(:,1),GrandObstacle(:,2));
        if isempty(uu)
            Sucess=0;
            break;
        end
        if mod(length(uu),2)==1
            %We may be inside something.
            Sucess=Sucess+1;
        end
    
    end
    if Sucess>3
        res=1;
    end
end
end

