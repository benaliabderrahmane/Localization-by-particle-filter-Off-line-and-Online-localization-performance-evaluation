function [K]=coefficients(Particles)
global N;
Et_x=sqrt(var(Particles.x));
Et_y=sqrt(var(Particles.y));
Et_theta=sqrt(var(Particles.theta));
Et=mean([Et_x,Et_y,Et_theta]);
if N > 50
    
    if Et> 5 
         K=1;
    else
         K=0.8;
    end
else 
    K=1;
end

end
