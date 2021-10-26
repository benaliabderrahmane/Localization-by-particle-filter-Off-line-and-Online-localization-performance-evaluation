function [weight] = likelihood(Choice,rho_rob,rho_particles)
%Ponderation and update weights:
% (function used in the main function ParticleFilter)

    Choice = upper(Choice);
    switch Choice 
        case "LIKELIHOOD1"
            diff=rho_rob-rho_particles;
            sigma=sqrt(var(diff));
            Mean=mean(diff);
            E=(diff-Mean)/sigma;
            weight=1/(sqrt(2*pi)*sigma)*exp(-0.5*E'*E);
        otherwise
            error("likelihood function is not defined!! add it in likelihood/likelihood.m")
    end
end
