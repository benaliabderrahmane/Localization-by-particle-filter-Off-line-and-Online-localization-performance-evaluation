function a = AngleWrap(a)


for jjjj=1:length(a)
%c'est la fonction pour rester dans l'interval -pi a pi sans passer a -3pi,...
while(a(jjjj)>pi && ~isinf(a(jjjj)))
    a(jjjj)=a(jjjj)-2*pi;
end

while(a(jjjj)<-pi && ~isinf(a(jjjj)))
    a(jjjj) = a(jjjj)+2*pi;
end
end
