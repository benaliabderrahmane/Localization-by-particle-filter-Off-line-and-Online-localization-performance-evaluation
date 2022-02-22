function PP = trajectoryGenerator(N,Obstacles,Start,End,Fig,flag)

    if(~isinBoxmap(Start(1),Start(2),Obstacles))
        error("start point isn't inside the box map")
    end
    if(~isinBoxmap(End(1),End(2),Obstacles))
        error("End point isn't inside the box map")
    end
    
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
        N1 = N;
        N2 = 0;
    elseif Flag == 2
        N1 = 0;
        N2 = N;
    else       
          N2 = round(1/(abs(Start(2)-End(2))/abs(Start(1)-End(1))+1)*N);
          N1 = N-N2;
         
    end

    if Flag == 1 || Flag == 3
 
        YminG = min(Start(2),End(2));
        YmaxG = max(Start(2),End(2));
        Dy = (YmaxG-YminG)/(N1);
        for i = 1:N1

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
            
%             figure(Fig)
%             plot([Xmin+0.75,Xmax-0.75],[Ymax,Ymax])
%             plot([Xmax-0.75,Xmax-0.75],[Ymax,Ymin])
%             plot([Xmax-0.75,Xmin+0.75],[Ymin,Ymin])
%             plot([Xmin+0.75,Xmin+0.75],[Ymin,Ymax])
            
            while(1)
                P=Particles_generator(Xmin+0.75,Xmax-0.75,Ymin,Ymax,-pi/2,pi/2, 1,Obstacles); % create a point in the rectangle 
                x= polyxpoly([PP(1,i) P(1)],[PP(2,i), P(2)],ObstaclesSegments(:,1),ObstaclesSegments(:,2)); % check for intersection
                if(isempty(x))
                PP(:,i+1)=P;   %add this point to PP    

                break
                end
            end     
        end
    end
    
    if Flag == 2 || Flag == 3
        
        XminG = min(Start(1),End(1));
        XmaxG = max(Start(1),End(1));
        Dx = (XmaxG-XminG)/(N2);
        
        for i = 1:N2

            %section 2 only 
            Xmin = XmaxG-(i)*Dx;
            Xmax = XmaxG-(i-1)*Dx;
            k = 1;
            y = zeros(1,25);
            for ii=1:217            
                if Obstacles(ii).Pos_vertex(2,1) > Xmin && Obstacles(ii).Pos_vertex(1,1) < Xmax
                    y(k) = Obstacles(ii).Centre(2);
                    k=k+1;
                end
                if Obstacles(ii).Pos_vertex(1,1) >= Xmin && Obstacles(ii).Pos_vertex(2,1) <= Xmax
                    y(k) = Obstacles(ii).Centre(2);
                    k=k+1;
                end
            end
            y = nonzeros(sort(y))';       
            Ymin = max(y(y<1));
            Ymax = min(y(y>1));
            
            if Ymax >2.3 && Xmax > 26
                Ymax = 2.3;
            end
            if isempty(Ymin)
                %condition to solve a error that occur sometimes (algorithm
                % can't find ymin so we take the last one)
                Ymin = temp;
            else 
                temp = Ymin;
            end
            
%             figure(Fig)
%             plot([Xmin,Xmax],[Ymax-0.75,Ymax-0.75])
%             plot([Xmax,Xmax],[Ymax-0.75,Ymin+0.75])
%             plot([Xmax,Xmin],[Ymin+0.75,Ymin+0.75])
%             plot([Xmin,Xmin],[Ymin+0.75,Ymax-0.75])
            
            while(1)
                P=Particles_generator(Xmin,Xmax,Ymin+0.75,Ymax-0.75,-pi/2,pi/2, 1,Obstacles); % create a point in the rectangle 
                x= polyxpoly([PP(1,i+N1) P(1)],[PP(2,i+N1), P(2)],ObstaclesSegments(:,1),ObstaclesSegments(:,2)); % check for intersection
                if(isempty(x))
                PP(:,i+N1+1)=P;  %add this point to PP  
                break
                end
            end     
        end
    end
    if flag
        figure(Fig)
        plot(PP(1,:),PP(2,:),'*k')
        hold on
        plot(PP(1,:),PP(2,:))
    end
end
          
  