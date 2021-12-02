function [iNextGeneration] = selection(Choice,weights,N)
%SELECTION of next generation particles
%   (used un main function ParticleFilter)
    Choice = upper(Choice);
    switch Choice
        case "ROULETTE WHEEL"
            CDF = cumsum(weights)/sum(weights); % vecteur des poids cumules et normalise
            %CD = [0.1 0.2 0.5 0.55 0.6 0.8 0.9 0.92 0.93 1]; for testing
            %only
            iSelect  = rand(N,1);
            % renvoie un vecteur qui ne contient que les bonnes particules et elimine le mauvaises et met au lieu d'eux l'indice des bonnes
            iNextGeneration = interp1(CDF,1:N,iSelect,'nearest','extrap');
        case "STOCHASTIC UNIVERSEL SAMPLING"
            CDF = cumsum(weights)/sum(weights); % vecteur des poids cumules et normalise
            %CD = [0.1 0.2 0.5 0.55 0.6 0.8 0.9 0.92 0.93 1]; for testing
            iSelect = linspace(0,1,N+1)+rand;
            iSelect(end) = []; %0 and 1 are the same thing so we delete one of them
            iSelectN = mod((iSelect),1)';%normalization
            % renvoie un vecteur qui ne contient que les bonnes particules et elimine le mauvaises et met au lieu d'eux l'indice des bonnes
            iNextGeneration = interp1(CDF,1:N,iSelectN,'nearest','extrap');
        otherwise
            error("selection function is not defined!! add it in selection/selection.m")
    end
end
