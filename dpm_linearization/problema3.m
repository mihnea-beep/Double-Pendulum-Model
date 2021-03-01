% date numerice

m = 4.81; 
l = 1.73;
zeta = 5.33;
g = 9.81;

params.m = m;
params.l = l;
params.zeta = zeta;
params.g = g;

Tmax = 100; % timp simulare
t = linspace(0, Tmax, 1e3).';

f = @(t, k) k.* double(t >= 0); % intrare treapta: k * 1(t)

% incarcare model
load_system('model3');
set_param('model3', 'StopTime', num2str(Tmax));

% magistrala
params_info = Simulink.Bus.createObject(params);
params_bus = evalin('base', params_info.busName);


%% liniarizarea sistemului pentru valori de regim stationar
% corespunzatoare intrarilor din ustar

% Determinare PSF si liniarizare
% Cautam PSF corespunzator valorilor din u* 

ustar = [0.3; 0.5; 1.5; 2.5; 3.4; 4.5];

for i = 1 : length (ustar);
    
    % SIM
    u = timeseries(f(t, ustar(i)), t);
    sim('model3'); % theta_out
    ystar_sim_cell{i} = theta_out.data(end, :);
    xstar_sim_cell{i} = x_out.data(end, :).';
    
    % Liniarizare    
    [A_sim_cell{i}, B_sim_cell{i}, C_sim_cell{i}, D_sim_cell{i}] = linmod('model3_pin', xstar_sim_cell{i}, ustar(i)); % u star sim

    % Simulare
    sys_sim_cell{i} = ss(A_sim_cell{i}, B_sim_cell{i}, C_sim_cell{i}, D_sim_cell{i});
    
    u2 = f(t, ustar(i)); % ustar
    x0 = zeros(4, 1); % vectorul de stare initiala
    ustar_repmat = repmat(ustar(i), length(t), 1);
    
    lin_sim_cell{i} = lsim(sys_sim_cell{i}, u2 - ustar_repmat, t, x0 - xstar_sim_cell{i}) + repmat(ystar_sim_cell{i}, length(t), 1);
    
    % TRIM - perechea xstar_trim, ustar_trim, ystar_trim
    [xstar_trim_cell{i}, ustar_trim_cell{i}, ystar_trim_cell{i}, ~] = trim('model3_pin', [], ustar(i), [], [], 1, []);
    
    % Liniarizare
    [A_trim_cell{i}, B_trim_cell{i}, C_trim_cell{i}, D_trim_cell{i}] = linmod('model3_pin', xstar_trim_cell{i}, ustar_trim_cell{i});
    
    % Simulare
    sys_trim_cell{i} = ss(A_trim_cell{i}, B_trim_cell{i}, C_trim_cell{i}, D_trim_cell{i});

    ustar_repmat = repmat(ustar(i), length(t), 1); % marirea dimensiunii lui ustar(i) pentru a corespunde lui u2
    
    lin_trim_cell{i} = lsim(sys_trim_cell{i}, u2 - ustar_repmat, t, x0 - xstar_trim_cell{i}) + repmat(ystar_trim_cell{i}', length(t), 1);
    
    if(real(eig(A_trim_cell{i})) >= 0)
        display('Valorile proprii au partea reala negativa');
    end
    
    % verificarea graficelor (NL (tema1), trim, sim)
     figure
     plot(t, lin_trim_cell{i}, '--r'); hold on
     plot(t, lin_sim_cell{i}, ':b'); hold on
     plot(theta_out.time, theta_out.data, '-k');
     xlim([0 15]);
     legend('trim', 'sim', 'tema 1');  
     title(['a) Iesirea sistemului in cazul u^{*} = ', num2str(ustar(i))]);
end

%% Observatii:
% Graficele obtinute in urma celor doua liniarizari aproape se confunda
% pentru fiecare intrare

%% caracteristica statica a modelului neliniar (din tema 1) si
% caracteristicile statice corespunzatoare sistemelor liniarizate obtinute
% la punctul anterior
st = double(t >= 0); % semnalul treapta

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
    %plot(t, lin_sim_cell{i}, ':b');             %% verificare pentru datele obtinute cu sim
    %hold on;
    % NL, sim, trim
    ystar1(i) = theta_out.data(end, 1);
    ystar1_sim_cell_caracteristica(i) = lin_sim_cell{i}(end, 1); % aceleasi date ca in ystar_sim/trim_cell{1}(:, 1),
    ystar1_trim_cell_caracteristica(i) = lin_trim_cell{i}(end, 1); % dar luate din graficul rezultat
    
end

plot(ustar1, ystar1, 'bx'), hold on % plotarea punctelor corespunzatoare regimului permanent
title('b) Caracteristica statica pentru theta1');
xlabel('u_{1}^{*}');
ylabel('y_{1}^{*}');

plot(ustar1, ystar1_sim_cell_caracteristica', 'k^'); % plotare puncte sim
hold on
plot(ustar1, ystar1_trim_cell_caracteristica', 'gs'); % plotare puncte trim
hold on

pol_theta1_static = polyfit(ustar1, ystar1, 3);
pol_theta1_sim_static = polyfit(ustar1, ystar1_sim_cell_caracteristica', 3);
pol_theta1_trim_static = polyfit(ustar1, ystar1_trim_cell_caracteristica', 3);


ustar_1_refined = ustar1(1) : 0.1 : ustar1(end);
ystar_1_refined = polyval(pol_theta1_static, ustar_1_refined);
ystar1_sim_cell_caracteristica_refined = polyval(pol_theta1_sim_static, ustar_1_refined);
ystar1_trim_cell_caracteristica_refined = polyval(pol_theta1_trim_static, ustar_1_refined);

plot(ustar_1_refined, ystar_1_refined, ':r');
hold on
plot(ustar_1_refined, ystar1_sim_cell_caracteristica_refined, '--b');
hold on
plot(ustar_1_refined, ystar1_trim_cell_caracteristica_refined, '-.k');
legend('puncte tema1', 'puncte sim', 'puncte trim', 'tema1 (NL)', 'sim', 'trim');

% caracteristica statica pentru theta_out.data(:, 2) - theta2

ustar2 = [0.3; 0.5; 1.5; 2.5; 3.4; 4.5]; % vectorul de amplificari pentru treapta (identic cu ustar1)
ystar2 = zeros(length(ustar2), 1);          

figure
for i = 1 : length(ustar2)
    
    u = timeseries(ustar2(i) .* st, t);
    sim('model3');
    
    %plot(theta_out.time, theta_out.data(:, 2)); %% verificarea momentelor inceputului regimului 
    %hold on;                                    %% permanent in cadrul fiecarei amplificari
    %plot(t, lin_trim_cell{i}(:, 2)); hold on
    %plot(t, lin_sim_cell{i}(:, 2)); hold on
    ystar2(i) = theta_out.data(end, 2);
    ystar2_sim_cell_caracteristica(i) = lin_sim_cell{i}(end, 2);
    ystar2_trim_cell_caracteristica(i) = lin_trim_cell{i}(end, 2);
    
end

plot(ustar2, ystar2, 'rs'), hold on % plotarea punctelor corspunzatoare regimului permanent
title('b) Caracteristica statica pentru theta2');
xlabel('u_{2}^{*}');
ylabel('y_{2}^{*}');
plot(ustar2, ystar2_sim_cell_caracteristica', 'k^'); % plotare puncte sim
hold on
plot(ustar2, ystar2_trim_cell_caracteristica', 'gs'); % plotare puncte trim
hold on

pol_theta2_static = polyfit(ustar2, ystar2, 4); 
pol_theta2_sim_static = polyfit(ustar2, ystar2_sim_cell_caracteristica', 4);
pol_theta2_trim_static = polyfit(ustar2, ystar2_trim_cell_caracteristica', 4);

ustar_2_refined = ustar2(1) : 0.1 : ustar2(end);
ystar_2_refined = polyval(pol_theta2_static, ustar_2_refined);
ystar2_sim_cell_caracteristica_refined = polyval(pol_theta2_sim_static, ustar_2_refined);
ystar2_trim_cell_caracteristica_refined = polyval(pol_theta2_trim_static, ustar_2_refined);

plot(ustar_2_refined, ystar_2_refined, 'r>');
hold on
plot(ustar_2_refined, ystar2_sim_cell_caracteristica_refined, '--b');
hold on
plot(ustar_2_refined, ystar2_trim_cell_caracteristica_refined, '-.k');
legend('puncte tema1', 'puncte sim', 'puncte trim', 'tema1 (NL)', 'sim', 'trim');

% "zoom out" pentru theta2 (restrangerea intervalului de afisare a
% caracteristicii)
figure
plot(ustar2, ystar2, 'rs'), hold on % plotarea punctelor corspunzatoare regimului permanent
title('Caracteristica statica pentru theta2 - "zoom out" ');
xlabel('u_{2}^{*}');
ylabel('y_{2}^{*}');
plot(ustar2, ystar2_sim_cell_caracteristica', 'k^'); % plotare puncte sim
hold on
plot(ustar2, ystar2_trim_cell_caracteristica', 'gs'); % plotare puncte trim
hold on

plot(ustar_2_refined, ystar_2_refined, 'r>');
hold on
plot(ustar_2_refined, ystar2_sim_cell_caracteristica_refined, '--b');
hold on
plot(ustar_2_refined, ystar2_trim_cell_caracteristica_refined, '-.k');
legend('puncte tema1', 'puncte sim', 'puncte trim', 'tema1 (NL)', 'sim', 'trim');
for i = 1 : length(ustar2)
    %plot(theta_out.time, theta_out.data(:, 2), '--b'); %% verificarea momentelor inceputului regimului 
    %hold on;                                    %% permanent in cadrul fiecarei amplificari
    %plot(t, lin_trim_cell{i}(:, 2), ':r'); hold on
    %plot(t, lin_sim_cell{i}(:, 2), '-k'); hold on
    %legend('NL', 'sim', 'trim'); 
end
xlim([0 10]);
ylim([-4*1e-3 4*1e-3]);

%% Observatii:
% Ca si in cadrul caracteristicii obtinute anterior (dpm_behaviour), caracteristica statica pentru theta1 este
% descrisa de ecuatia unei drepte, pe intervalul de valori admise. Atat
% graficul rezultat utilizand liniarizarea simularii obisnuite, dar si cel
% al modelului cu punctele statice de functionare aflate cu functia trim
% sunt aproape identice.

% In cazul iesirii theta2, graficul obtinut cu ajutorul metodei trim pentru
% caracteristica statica, este descris de ecuatia unei drepte, pe cand
% caracteristica obtinuta prin simulare obisnuita si liniarizare se
% aseamana cu cea a modelului neliniar (din prima tema). Prin plotare
% separata, pe un interval diferit de timp, sau in raport cu un alt grafic
% (spre exemplu, in secventele comentate de verificare), se poate observa ca
% distantele dintre puncte sunt foarte mici (de ordinul 10^-7), iar cu un 
% "zoom out" acestea au aspectul unei drepte.
% Astfel, daca limitam intervalul de afisare la niste valori mai "mari" 
% relativ la ordinul marimii din graficul initial (10^-7), caracteristicile 
% devin drepte