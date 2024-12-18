%% Animazione del drone con dati dalla simulazione
% Creazione della figura
figure;
axis equal;   % Assi uguali per mantenere le proporzioni
grid on;
hold on;
title('Animazione del Drone');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
xlim([-1 1]); ylim([-1 1]); zlim([0 2]); % Limiti statici della scena
view(3);     % Vista 3D

% Parametri della figura del drone
arm_length = 0.2; % Lunghezza dei bracci del drone
drone_body = [0, 0, 0;  % Centro del drone
              arm_length, 0, 0;  % Estensione lungo X positivo
              -arm_length, 0, 0; % Estensione lungo X negativo
              0, arm_length, 0;  % Estensione lungo Y positivo
              0, -arm_length, 0]; % Estensione lungo Y negativo

% Estrazione dei dati di simulazione
roll = x(:, STATE.PHI);   % Roll (phi)
pitch = x(:, STATE.THETA);% Pitch (theta)
yaw = x(:, STATE.PSI);    % Yaw (psi)
z = x(:, STATE.Z);        % Posizione verticale (quota)
x_pos = x(:, STATE.X);    % Posizione lungo X
y_pos = x(:, STATE.Y);    % Posizione lungo Y

% Animazione del drone
for i = 1:length(t)
    % Matrice di rotazione basata sugli angoli di Eulero
    R = eul2rotm([yaw(i), pitch(i), roll(i)], 'ZYX');
    
    % Trasformazione del corpo del drone
    drone_rotated = (R * drone_body')';  % Rotazione
    drone_translated = drone_rotated + [x_pos(i), y_pos(i), z(i)]; % Traslazione

    % Pulizia della scena precedente
    cla;

    % Disegna i bracci del drone
    plot3([drone_translated(1,1), drone_translated(2,1)], ...
          [drone_translated(1,2), drone_translated(2,2)], ...
          [drone_translated(1,3), drone_translated(2,3)], 'r', 'LineWidth', 2); % Braccio X+
    plot3([drone_translated(1,1), drone_translated(3,1)], ...
          [drone_translated(1,2), drone_translated(3,2)], ...
          [drone_translated(1,3), drone_translated(3,3)], 'g', 'LineWidth', 2); % Braccio X-
    plot3([drone_translated(1,1), drone_translated(4,1)], ...
          [drone_translated(1,2), drone_translated(4,2)], ...
          [drone_translated(1,3), drone_translated(4,3)], 'b', 'LineWidth', 2); % Braccio Y+
    plot3([drone_translated(1,1), drone_translated(5,1)], ...
          [drone_translated(1,2), drone_translated(5,2)], ...
          [drone_translated(1,3), drone_translated(5,3)], 'k', 'LineWidth', 2); % Braccio Y-

    % Disegna il punto centrale del drone
    plot3(drone_translated(1,1), drone_translated(1,2), drone_translated(1,3), ...
          'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

    % Aggiorna la scena
    pause(0.05); % Ritardo per sincronizzare l'animazione
end

