function a = AngleWrap(a)
%c'est la fonction pour rester dans l'interval -pi a pi sans passer a -3pi,...
if(a>pi)
    a=a-2*pi;
elseif(a<-pi)
    a = a+2*pi;
end;
