clear all; clc; close all
%% c) Compararea iesirii sistemului neliniar cu iesirea sitemului liniar
% interpolat (B-spline, laborator 6) 

%%
m = 4.81; 
l = 1.73;
zeta = 5.33;
g = 9.81;

params.m = m;
params.l = l;
params.zeta = zeta;
params.g = g;

T = 100;                           % timp de simulare 
Tmax = T;
timp = linspace(0,T,1e3).';         % interval de simulare
t = timp;

ustar = [1; 6; 10];            % puncte in care se va face liniarizarea 

f = @(t, k) k.* double(t >= 0); % intrare treapta: k * 1(t)

% selectia de liniarizare pe intervale de timp
sigma = 1* double(t >= 20) + 4 * double((20 < t).*(t <= 50)) + 5 * double(t <= T);

%% obtinerea punctului de echilibru si a liniarizarilor

% incarcare model
load_system('model3');
set_param('model3', 'StopTime', num2str(Tmax));

% magistrala
params_info = Simulink.Bus.createObject(params);
params_bus = evalin('base', params_info.busName);


% sistemul este suficient de simplu pentru a ne permite sa calculam explicit PSF-urile si matricele de stare
for i = 1 : length (ustar)
    
    % SIM
  
    u = timeseries(f(t, ustar(i)), t);
    sim('model3'); % theta_out
    ystar_sim_cell{i} = theta_out.data(end, :);
    xstar_sim_cell{i} = x_out.data(end, :).';
    
    % Liniarizare    
    [A_sim_cell{i}, B_sim_cell{i}, C_sim_cell{i}, D_sim_cell{i}] = linmod('model3_pin', xstar_sim_cell{i}, ustar(i)); % u star sim

    % Simulare
    sys_sim_cell{i} = ss(A_sim_cell{i}, B_sim_cell{i}, C_sim_cell{i}, D_sim_cell{i});
    
    u2 = f(t, ustar(i));
    x0 = zeros(4, 1);
    ustar_repmat = repmat(ustar(i), length(t), 1);
    lin_sim_cell{i} = lsim(sys_sim_cell{i}, u2 - ustar_repmat, t, x0 - xstar_sim_cell{i}) + repmat(ystar_sim_cell{i}, length(t), 1);
    
    % TRIM - perechea xstar_trim, ustar_trim, ystar_trim
    [xstar_trim_cell{i}, ustar_trim_cell{i}, ystar_trim_cell{i}, ~] = trim('model3_pin', [], ustar(i), [], [], 1, []);
    [A_trim_cell{i}, B_trim_cell{i}, C_trim_cell{i}, D_trim_cell{i}] = linmod('model3_pin', xstar_trim_cell{i}, ustar_trim_cell{i});
    sys_trim_cell{i} = ss(A_trim_cell{i}, B_trim_cell{i}, C_trim_cell{i}, D_trim_cell{i});
    
    u2 = f(t, ustar(i));
    x0 = zeros(4, 1);
    ustar_repmat = repmat(ustar(i), length(t), 1);
    
    lin_trim_cell{i} = lsim(sys_trim_cell{i}, u2 - ustar_repmat, t, x0 - xstar_trim_cell{i}) + repmat(ystar_trim_cell{i}', length(t), 1);

    xstar{i} = xstar_sim_cell{i};
    ystar{i} = ystar_sim_cell{i};
    A{i} = A_sim_cell{i};
    B{i} = B_sim_cell{i};
    C{i} = C_sim_cell{i};
    D{i} = D_sim_cell{i};
    
    if(real(eig(A_sim_cell{i})) >= 0)
        display('Valori proprii pozitive!');
    end
end

%% simulare sistemului liniarizat pe portiuni
% Intrare
u = double(t>=0);
u = 0.55 * double(timp<=20) + 3.57 * double((20<timp).*(timp<=70)) + 4.650 * double((70<timp).*(timp<=T));
% plecam din starea initiala nula
xinitial = zeros(4, 1);                

y_int=[];

% gasim momentele de timp la care are loc o schimbare a selectiei liniarizarii active
alternanta=find((diff(sigma))~=0); 
% la care adaugam primul si ultimul moment de timp 
alternanta=[1 alternanta' length(timp)];

% constructie ponderi de tipul B-spline
d=3; % alegand acesta valoare vom avea continuitate pana la a 3a derivata
m=1; % m+1 este numarul de functii B-spline ce vor fi obtinut
knotv=timp([alternanta(1)*ones(1,d-1) alternanta alternanta(end)*ones(1,d-1)]);
% compute the bsplines of order d and less
bv=bsplinesSymbolic(d,knotv);

[btt,tt]=plot_bsplines(bv{end},knotv); % returneaza valorile numerice si intervalul de timp

% construiesc ponderile (combin primele 2 si ultime 2 functii spline, vreau
% ca pentru acestea sa am valori de 1
tt(end)=[];
temp=[btt{1}+btt{2}; reshape([btt{3:end-2}],length(btt)-4,length(tt)); btt{end-1}+btt{end}];

figure; hold on; grid on
for i=1:size(temp,1)
    lambda(i,:)=interp1(tt,temp(i,:),timp);
    plot(timp,lambda(i,:))
end

%%
y_int=[];
for i=1:length(ustar) % daca sunt n elemente in vector inseamna ca avem n-1 intervale    
    [ylin,~,xlin]=lsim(ss(A{i},B{i},C{i},D{i}), u - repmat(ustar(i), length(t), 1), timp, xinitial - xstar{i}); % se simuleaza raspunsul sistemului liniarizat la intrarea ustar(lin)
    y_lin=ylin+ystar{i}; % stocam iesirea pentru fiecare sistem in parte
    if i==1
        y_int=y_lin'.*lambda(i,:);
    else
        y_int=y_int+y_lin'.*lambda(i,:);
    end
end

%% simulare sistem neliniar 
load_system('model3');  %incarcam in memorie modelul simulink
set_param('model3', 'StopTime', num2str(T)) % setam timpul de simulare
% 
u=timeseries(u',timp);       % construim structura ce este primita de blocul From Workspace
sim('model3');       % rulam modelul simulink, la terminarea simularii avem stocata iesirea in structura ysim

%% plotare rezultate
figure; 
plot(theta_out.Time, theta_out.Data,'--b'); hold on
plot(timp,y_int, ':r'); hold on
display(alternanta);
% display(y_int');
% display(theta_out);
%legend('ysim',"y_{int}");
legend('','Tema 1 (NL)', '','Interpolare');
title('Iesiri sistem NL si sistem liniar interpolat');
% Pentru aceeasi intrare, sistemul interpolat are valori foarte apropiate
% de cele ale sistemului neliniar.


