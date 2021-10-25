figure(122)
N=[50 100 200 300 400 500 600 700 800 1000];
TC=[27.71 40.03 73.71 103.41 145.29 171.15 254.23 297.21 380 468];
IC=[60 48 45 42 40 39 39 36 37 33];
subplot(211)
plot(N,TC)
xlabel('Nombre de particules');
ylabel('Temps de convergence [s]');
title('(a) ')

subplot(212)
plot(N,IC)
xlabel('Nombre de particules');
ylabel('Iteration de convergence');
title('(b) ')
