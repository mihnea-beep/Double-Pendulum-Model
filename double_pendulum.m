% date numerice

m = 4.81; 
l = 1.73;
zeta = 5.33;
g = 9.81;

params.m = m;
params.l = l;
params.zeta = zeta;
params.g = g;

%% 

% implementarea modelului matematic in simulink si ilustrarea
% raspunsului la o intrare treapta - model3.slx

% magistrala

params_info = Simulink.Bus.createObject(params);
params_bus = evalin('base', params_info.busName);

Tmax = 30; % timp simulare

load_system('model3');
set_param('model3', 'StopTime', num2str(Tmax));

t = linspace(0, Tmax, 100).';
st = double(t >= 0); % semnalul treapta

u = timeseries(st, t);

sim('model3');

% ilustrarea raspunsului sistemului la intrarea treapta

figure
plot(theta_out);
title('a) Raspuns la intrare treapta');
legend('theta1', 'theta2');
ylabel('theta1, theta2');
xlabel('timp');
hold on;

%%  

% determinarea caracteristicii statice a modelului prin amplificari ale treptei

% caracteristica statica pentru theta_out.data(:, 1) - theta1

ustar1 = [0.3; 0.5; 1.5; 2.5; 3.4; 4.5]; % vectorul de amplificari pentru treapta
ystar1 = zeros(length(ustar1), 1);

figure


for i = 1 : length(ustar1)
    
    u = timeseries(ustar1(i) .* st, t);
    sim('model3');
    
    %plot(theta_out.time, theta_out.data(:, 1)); %% verificarea inceputului regimului permanent 
    %hold on;                                    %% in cadrul fiecarei amplificari
    
    ystar1(i) = theta_out.data(end, 1);

end


plot(ustar1, ystar1, 'bp'), hold on % plotarea punctelor corspunzatoare regimului permanent
title('Caracteristica statica pentru theta1');
xlabel('u1*');
ylabel('y1*');

pol_theta1_static = polyfit(ustar1, ystar1, 3);

ustar_1_refined = ustar1(1) : 0.1 : ustar1(end);
ystar_1_refined = polyval(pol_theta1_static, ustar_1_refined);

plot(ustar_1_refined, ystar_1_refined, '--m');

% caracteristica statica pentru theta_out.data(:, 2) - theta2

ustar2 = [0.3; 0.5; 1.5; 2.5; 3.4; 4.5]; %% vectorul de amplificari pentru treapta (identic cu ustar1)
ystar2 = zeros(length(ustar2), 1);          

figure
for i = 1 : length(ustar2)
    
    u = timeseries(ustar2(i) .* st, t);
    sim('model3');
    
    %plot(theta_out.time, theta_out.data(:, 2)); %% verificarea momentelor inceputului regimului 
    %hold on;                                    %% permanent in cadrul fiecarei amplificari
    
    ystar2(i) = theta_out.data(end, 2);

end


plot(ustar2, ystar2, 'rs'), hold on % plotarea punctelor corspunzatoare regimului permanent
title('Caracteristica statica pentru theta2');
xlabel('u2*');
ylabel('y2*');

pol_theta2_static = polyfit(ustar2, ystar2, 4);

ustar_2_refined = ustar2(1) : 0.1 : ustar2(end);
ystar_2_refined = polyval(pol_theta2_static, ustar_2_refined);

plot(ustar_2_refined, ystar_2_refined, '--m');

%% Observatii: 
%
% Caracteristica statica pentru theta2 seamana
% cu o dreapta (vizibil, atunci cand in graficul caracteristicii 
% apar si graficele iesirilor, iar valorile vizibile pe grafic
% cresc cu mai multe ordine).

% Plaja de valori corespunzatoare unei intrari valide (pentru care sistemul
% se stabilizeaza) este marginita inferior de 0 (desi
% exista comenzi negative pentru care sistemul se stabilizeaza, am considerat
% ca intrarile sa aiba sens doar daca sunt strict pozitive) si 
% superior de minim u* = 120. Aceasta este ultima valoare 
% pentru care am vazut ca sistemul ajnge in regim stationar. 
% Pentru valori mai mari de 1500, am observat ca sistemul nu se stabilizeaza.
% Spre exemplu, pentru o amplificare de 1500, iesirea este o dreapta de ec.
% aproximativ y = 10 * x. 

% Pentru valori intermediare [120:1500], apar warning-uri
% referitoare la step size si executia se opreste cu erori
% pentru valorile aflate in integrator. Am modificat setarile din cadrul
% solverului, insa aceasta eroare persista. Am ajuns la concluzia ca in
% integrator apar numere foarte mari, din cauza impartirii la cos (a carui
% valoare poate fi foarte mica).
% Pendulul dublu reprezinta un sistem a carui
% miscare (in functie de param. fizici care il descriu) poate fi haotica.

%% 
% comportamentul sistemului la aceeasi intrare treapta
% dar cu valori diferite ale starii initiale - model3c.slx

% conditiile initiale

init_cond1 = [0; 0.1; 0.23; 0.4; 1; 1.2; 1.5]; % valori conditii initiale pentru primele integratoare
init_cond2 = [0; 0.2; 0.35; 0.45; 0.9; 1.1; 1.4]; 

x0_theta1 = 0; % la inceput, conditiile initiale sunt nule
x0_theta2 = 0;

u = timeseries(st, t); % treapta

load_system('model3c');
set_param('model3c', 'StopTime', num2str(Tmax));
sim('model3c');

figure
title('c) Schimbarea comportamentului in functie de conditiile initiale - theta1');

for i = 1 : length(init_cond1)
    
    x0_theta1 = init_cond1(i);
    
    sim('model3c');
    plot(theta_out.time, theta_out.data(:, 1));
    hold on;
    
    
end

xlabel('Timp');
ylabel('Variatii theta1');

figure
title('c) Schimbarea comportamentului in functie de conditiile initiale - theta2');

for i = 1 : length(init_cond2)
    
    x0_theta2 = init_cond2(i);
    
    sim('model3c');
    plot(theta_out.time, theta_out.data(:, 2));
    hold on;
    
    
end

xlabel('Timp');
ylabel('Variatii theta2');

%%
% Pendulul dublu este un sistem dinamic, cu o puternica sensibilitate
% la conditiile initiale.
% Se observa faptul ca, indiferent de variatiile valorilor acestora, sistemul intra
% in regim permanent in aceleasi valori (atat theta1, cat si
% theta2)
