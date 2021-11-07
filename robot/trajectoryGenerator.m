function [PP, v ] = trajectoryGenerator(N,Obstacles,Start,End,Fig)

    PP = zeros(3,N+2); % N point + start + end
    PP(:,1) = Start;
    PP(:,end) = End;

    Flag = 0; % 1 trajectory in section 1 only
              % 2 trajectory in section 2 only 
              % 3 trajectory in both sections
              
    Sobst=size(Obstacles);
    ObstaclesSegments=zeros(Sobst(2)*3,2); % this variable contains the segment of the map         
    for iii=1:Sobst(2)
        % used to check in the trajectory is in inside the map
        kk = (iii-1)*3;
        ObstaclesSegments(kk+1:kk+2,:)=[Obstacles(iii).Pos_vertex(:,1)  Obstacles(iii).Pos_vertex(:,2)];
        ObstaclesSegments(kk+3,:)=[NaN NaN];
    end
    
    if Start(2)>2.3  || End(2)>2.3
        Flag = Flag +1;
    end
    if Start(2)<=2.3 || End(2)<=2.3
        Flag = Flag +2;
    end
    
    if Flag == 1
        
        YminG = min(Start(2),End(2));
        YmaxG = max(Start(2),End(2));
        Dy = (YmaxG-YminG)/(N);
        
        for i = 1:N

            %section 1 only 
            Ymin = YmaxG-(i)*Dy;
            Ymax = YmaxG-(i-1)*Dy;
            k = 1;
            x = zeros(1,25);
            for ii=1:217
                if Obstacles(ii).Pos_vertex(2,2) > Ymin && Obstacles(ii).Pos_vertex(1,2) < Ymax
                    x(k) = Obstacles(ii).Centre(1);
                    k=k+1;
                end
                if Obstacles(ii).Pos_vertex(1,2) > Ymin && Obstacles(ii).Pos_vertex(2,2) < Ymax
                    x(k) = Obstacles(ii).Centre(1);
                    k=k+1;
                end
            end
            x = nonzeros(sort(x))';
            Xmin = max(x(x<27));
            Xmax = min(x(x>27));
            plot([Xmin,Xmax],[Ymax,Ymax])
            plot([Xmax,Xmax],[Ymax,Ymin])
            plot([Xmax,Xmin],[Ymin,Ymin])
            plot([Xmin,Xmin],[Ymin,Ymax])
            
            while(1)
                P=Particles_generator(Xmin,Xmax,Ymin,Ymax,-pi/2,pi/2, 1,Obstacles); % create a point in the rectangle 
                x= polyxpoly([PP(1,i) P(1)],[PP(2,i), P(2)],ObstaclesSegments(:,1),ObstaclesSegments(:,2)); % check for intersection
                if(isempty(x))
                PP(:,i+1)=P;  % ajout de ce point dans le vecteur PP  
                plot(P(1),P(2),'*r')
                break
                end
            end     
        end
    end
figure(Fig)
plot(PP(1,:),PP(2,:),'*k')
hold on
plot(PP(1,:),PP(2,:),'k')     
     v= 0.6;       
end
          
  