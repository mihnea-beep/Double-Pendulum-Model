# Double-Pendulum-Model

### Schema pendulului dublu

[comp](./pictures/pendule.png)

### Modelul matematic:

- [comp](./pictures/model.png)
- [comp](./pictures/model2.png)

Iesirile sunt y1(t) = theta1(t) si y2(t) = theta2(t)

### dpm_behaviour

- Implementarea modelului matematic intr-o schema Simulink si ilustrarea raspunsului
la o intrare treapta (printr-un script Matlab ce apeleaza schema Simulink).

- Ilustrarea caracteristicii statice a modelului (prin apelari repetate ale modelului
  intr-un script Matlab) => y1 = y1(u) si y2 = y2(u)

- Determinarea unui interval de valori ale intrarii pentru care modelul admite regim
stationar

- Ilustrarea comportamentului sistemului la o aceeasi intrare treapta cu diferite
valori ale starii initiale

### dpm_linearization

- Liniarizarea sistemului pentru valorile de regim stationar corespunzatoare intrarilor
u(t) din {u1*, u2*, ... };

- Ilustrarea caracteristicii statice a modelului liniar obtinut anterior (dpm_behaviour)
si a caracteristicililor statice corespunzatoare noilor sisteme liniarizate

- Compararea iesirii sistemului neliniar cu iesirea sistemului liniar interpolat
(utilizand functii *B-spline*)
