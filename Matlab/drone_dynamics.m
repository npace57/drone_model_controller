%% Parametri del sistema
% Costanti fisiche e parametri del drone
m = 1.0;              % Massa del drone in kg
g = 9.81;             % Accelerazione gravitazionale in m/s^2
k_t = 1.0;            % Coefficiente di spinta (relazione velocità rotore-spinta)
k_q = 0.1;            % Coefficiente di coppia (relazione velocità rotore-coppia)
I = diag([0.02, 0.02, 0.04]); % Tensore d'inerzia (diagonale: Ixx, Iyy, Izz)
L = 0.2;              % Lunghezza del braccio (distanza dal centro ai rotori)

%% Indici degli stati (usando una struct per chiarezza)
STATE = struct('PHI', 1, 'THETA', 2, 'PSI', 3, ...
               'OMEGA_X', 4, 'OMEGA_Y', 5, 'OMEGA_Z', 6, ...
               'Z', 7, 'DZ', 8, 'X', 9, 'DX', 10, 'Y', 11, 'DY', 12);

%% Condizioni iniziali
x0 = zeros(12, 1); % Tutti gli stati iniziali a zero
z_target = 1.0;    % Altezza obiettivo in metri
rotation_impulse_time = 2.0; % Tempo per applicare l'impulso di rotazione dopo 1 secondo di hovering

%% Tempo di simulazione
tspan = [0 10];  % Simula da 0 a 10 secondi

%% Simulazione con ODE45
[t, x] = ode45(@(t, x) droneDynamics(t, x, STATE, m, g, k_t, k_q, I, L, z_target, rotation_impulse_time), tspan, x0);

%% Visualizzazione dei risultati
figure;
% Posizione verticale
subplot(3,1,1);
plot(t, x(:,STATE.Z)); grid on;
title('Posizione verticale (z)');
xlabel('Tempo [s]');
ylabel('Altezza [m]');
legend('z');

% Angoli di Eulero
subplot(3,1,2);
plot(t, x(:,STATE.PHI:STATE.PSI)); grid on;
title('Angoli di Eulero (Roll, Pitch, Yaw)');
xlabel('Tempo [s]');
ylabel('Angoli [rad]');
legend('Roll (\phi)', 'Pitch (\theta)', 'Yaw (\psi)');

% Posizione orizzontale
subplot(3,1,3);
plot(t, x(:,STATE.X), t, x(:,STATE.Y)); grid on;
title('Posizione orizzontale (x, y)');
xlabel('Tempo [s]');
ylabel('Posizione [m]');
legend('x', 'y');

%% Funzione dinamica del drone
function dx = droneDynamics(t, x, STATE, m, g, k_t, k_q, I, L, z_target, rotation_impulse_time)
    % Estrarre stati attuali
    phi = x(STATE.PHI);        % Rollio
    theta = x(STATE.THETA);    % Beccheggio
    psi = x(STATE.PSI);        % Imbardata
    omega_x = x(STATE.OMEGA_X); % Velocità angolare attorno a x
    omega_y = x(STATE.OMEGA_Y); % Velocità angolare attorno a y
    omega_z = x(STATE.OMEGA_Z); % Velocità angolare attorno a z
    z = x(STATE.Z);            % Posizione verticale
    dz = x(STATE.DZ);          % Velocità verticale

    % Controller per la posizione verticale
    Kp_z = 5;  % Guadagno proporzionale per z
    Kd_z = 2;  % Guadagno derivativo per z

    % Spinta totale desiderata
    F_total = m * g + Kp_z * (z_target - z) - Kd_z * dz;

    % Assicurarsi che la spinta sia positiva
    F_total = max(F_total, 0);

    % Torques
    tau_x = 0; % Nessuna coppia di Roll
    tau_y = 0; % Nessuna coppia di Pitch inizialmente
    tau_z = 0; % Nessuna coppia di Yaw

    % Applicare un impulso di Pitch
    if t >= rotation_impulse_time && t < rotation_impulse_time + 0.5
        tau_y = 0.2; % Coppia di Pitch
    end

    % Calcolo delle accelerazioni angolari
    tau = [tau_x; tau_y; tau_z];
    omega = [omega_x; omega_y; omega_z];
    domega = I \ (tau - cross(omega, I * omega));

    % Accelerazione verticale
    ddz = (F_total / m) - g;

    % Accelerazioni orizzontali
    ddx = (F_total / m) * sin(theta); % Forza lungo l'asse x
    ddy = (F_total / m) * cos(theta) * sin(phi); % Forza lungo l'asse y

    % Matrice cinematica per gli angoli di Eulero
    J_inv = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
             0, cos(phi), -sin(phi);
             0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    euler_dot = J_inv * [omega_x; omega_y; omega_z];

    % Derivate degli stati
    dx = zeros(12,1);
    dx(STATE.PHI) = euler_dot(1);     % Derivata del Rollio
    dx(STATE.THETA) = euler_dot(2);   % Derivata del Beccheggio
    dx(STATE.PSI) = euler_dot(3);     % Derivata dell'Imbardata
    dx(STATE.OMEGA_X) = domega(1);    % Accelerazione angolare attorno a x
    dx(STATE.OMEGA_Y) = domega(2);    % Accelerazione angolare attorno a y
    dx(STATE.OMEGA_Z) = domega(3);    % Accelerazione angolare attorno a z
    dx(STATE.Z) = dz;                 % Velocità verticale
    dx(STATE.DZ) = ddz;               % Accelerazione verticale
    dx(STATE.X) = x(STATE.DX);        % Velocità orizzontale x
    dx(STATE.DX) = ddx;               % Accelerazione orizzontale x
    dx(STATE.Y) = x(STATE.DY);        % Velocità orizzontale y
    dx(STATE.DY) = ddy;               % Accelerazione orizzontale y

    % Debugging delle forze e accelerazioni
    fprintf('Time: %.2f, ddx: %.2f, ddy: %.2f, phi: %.2f, theta: %.2f\n', ...
        t, ddx, ddy, phi, theta);
end
