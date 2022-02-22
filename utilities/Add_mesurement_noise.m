function [ Xn, Yn, Dn ] = Add_mesurement_noise( Xrob, Yrob, Xmesure, Ymesure, Portee, Bruit)
%ADD_MESUREMENT_NOISE Summary of this function goes here
%   Detailed explanation goes here

Dmesure=sqrt((Xmesure-Xrob)^2+(Ymesure-Yrob)^2);


if Bruit > 0
    if Portee < 5
        if Dmesure<1
            Dn=(0.01)*rand('double')-0.005+Dmesure;
        else
            Dn=min(Dmesure-Dmesure/100+(Dmesure/50)*rand('double'),Portee);
        end
    else
        if Dmesure<10
            Dn=(0.01)*rand('double')-0.005+Dmesure;
        else
            Dn=(0.03)*rand('double')-0.015+Dmesure;
        end
    end
else
    Dn=Dmesure;
end

Xn=((Xmesure-Xrob)*(Dn/(Dmesure)))+Xrob;
Yn=((Ymesure-Yrob)*(Dn/(Dmesure)))+Yrob;

end

