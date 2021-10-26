function [weight] = likelihood(rho_rob,rho_particles)
%Ponderation et calcul des nouveaux poids :
% (function used in the main function ParticleFilter)
    diff=rho_rob-rho_particles;
    sigma=sqrt(var(diff));
    Mean=mean(diff);
    E=(diff-Mean)/sigma;
    weight=1/(sqrt(2*pi)*sigma)*exp(-0.5*E'*E);
end

