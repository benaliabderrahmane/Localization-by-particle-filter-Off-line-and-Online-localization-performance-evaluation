function [iNextGeneration] = selection(Choice,weights,N)
%SELECTION of next generation particles
%   (used un main function ParticleFilter)
    Choice = upper(Choice);
    switch Choice 
        case "SELECTION1"
            CDF = cumsum(weights)/sum(weights); % vecteur des poids cumules et normalise
            iSelect  = rand(N,1);
            % renvoie un vecteur qui ne contient que les bonnes particules et elimine le mauvaises et met au lieu d'eux l'indice des bonnes
            iNextGeneration = interp1(CDF,1:N,iSelect,'nearest','extrap');
        otherwise
            error("selection function is not defined!! add it in selection/selection.m")
    end
end
