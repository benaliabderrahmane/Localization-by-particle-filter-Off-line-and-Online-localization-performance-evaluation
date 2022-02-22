
function [ Robot_Fin ] = Run_Robot( Robot, u, dt, bruit )
%RUN_ROBOT Sdeplace le robot suivant u et dt pour 0.1s
%   Robot : Avec la posisiton x,y,thêta
% U vitesse linéaire
% dt vitesse angulaire
% bruit : flag 0 robot bruit=robot parfait : 1 robot = robot bruité.
Robot_Fin=Robot;
Duree=0.1;
%Ajout duree boucle de commande.
L=0.3562;

sd=(dt*L+2*u)/2;
sg=(2*u-dt*L)/2;


bd=4.523070446093282e-4;
bg=4.521701688771833e-4;

if bruit==0
    sdb=sd;
    sgb=sg;
else
    sdb=(rand()*2-1)*bd*sd+sd;
    sgb=(rand()*2-1)*bg*sg+sg;
end

ub=((sdb+sgb)/2)*Duree;
dtb=((sdb-sgb)/L)*Duree;

Robot_Fin.x=Robot_Fin.x+ub*cos(Robot_Fin.theta+dtb/2);
Robot_Fin.y=Robot_Fin.y+ub*sin(Robot_Fin.theta+dtb/2);
Robot_Fin.theta=Robot_Fin.theta+dtb;

end
