clc
clear all
close all


rosport = 11311;
rosip = "192.168.1.56";

try 
    rosinit(rosip,rosport)
catch e
    rosshutdown
    rosinit(rosip,rosport)
end

%% MAPPA E PARAMETRI

% trascrizione mappa
img = imread('rotated_maze.pgm');
map = img < 250;

% plot di sicurezza
figure;
imagesc(map);
colormap(gray);
axis equal tight;

mapSize = size(map);
odom_sub = rossubscriber('/odom', 'nav_msgs/Odometry');
odom_msg = receive(odom_sub, 3);
position = odom_msg.Pose.Pose.Position;
ori = odom_msg.Pose.Pose.Orientation;
[x_start, y_start] = coordToCell(position.X, position.Y, map);
start = [y_start, x_start];
goal = [90, 85];

%% A* + smoothing
% [v, w] = function a*_smooth(map,start,goal, ...)
raw_path = a_star(map, start, goal);
path = raw_path([true; any(diff(raw_path),2)], :);
shortcut_path = path(1,:);
i = 1;
while i < size(path,1)
    j = min(i+10, size(path,1));
    while j > i+1
        if is_line_free(path(i,:), path(j,:), map)
            break;
        end
        j = j - 1;
    end
    shortcut_path = [shortcut_path; path(j,:)];
    i = j;
end

t = 1:size(shortcut_path,1);
tt = linspace(1, size(shortcut_path,1), 200);
xs = pchip(t, shortcut_path(:,1)', tt);
ys = pchip(t, shortcut_path(:,2)', tt);
smoothed_path = [xs', ys'];

% Rimuovi punti che passano dentro ostacoli statici
clean_path = [];
for i = 1:size(smoothed_path,1)
    pt = round(smoothed_path(i,:));
    if all(pt >= 1) && all(pt <= mapSize) && map(pt(1), pt(2)) == 0
        clean_path = [clean_path; smoothed_path(i,:)];
    end
end

%% PLOT
figure;
imshow(1-map, 'InitialMagnification', 800); hold on;
plot(start(2), start(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(goal(2), goal(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
plot(clean_path(:,2), clean_path(:,1), 'k--', 'LineWidth', 1.5);
h_robot = plot(start(2), start(1), 'r.', 'MarkerSize', 20);
title('Evitamento Statico + Dinamico Naturale');
legend('Start', 'Goal', 'Path', 'Robot', 'Ostacoli Dinamici', 'Location','northeastoutside');
grid on;
%% 
% Parametri
map_resolution = 0.05; % metri per cella (modifica se necessario)
K_v = 1000;  % guadagno per velocità lineare
K_w = -1;  % guadagno per velocità angolare

vel_pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
vel_msg = rosmessage(vel_pub);

% Converte path in coordinate reali
clean_path_m = (clean_path * map_resolution);

% Loop principale
for i = 1:(size(clean_path_m, 1)-1)
    target = clean_path_m(i+1, :)'; % Punto target [x; y]
    error_theta = 0;
    % Estrai il quaternione
    quat = [odom_msg.Pose.Pose.Orientation.W, ...
        odom_msg.Pose.Pose.Orientation.X, ...
        odom_msg.Pose.Pose.Orientation.Y, ...
        odom_msg.Pose.Pose.Orientation.Z];

    % Convertilo in angoli di Eulero [roll, pitch, yaw]
    eul = quat2eul(quat);

    % L'angolo di yaw è il terzo valore
    yaw = eul(3);

    
    % Recupera posizione corrente del robot (qui va integrato un listener per stimare pose reale)
    % Per ora, simuliamo posizione attuale come il punto corrente del path:
    current_pose = [clean_path_m(i, 1); clean_path_m(i, 2); yaw]; % [x; y; theta]
    
    % Calcola direzione e distanza
    delta = target(1:2) - current_pose(1:2);
    rho = norm(delta);
    theta_target = atan2(delta(2), delta(1));
    error_theta = wrapToPi(theta_target - current_pose(3));

    % Controller proporzionale
    v = K_v * rho;
    w = K_w * error_theta;
    
    % Costruisci e pubblica messaggio
    vel_msg.Linear.X = v;
    vel_msg.Angular.Z = w;
    send(vel_pub, vel_msg);
    
    % Debug info
    fprintf("Target: [%.2f %.2f] | v = %.2f, w = %.2f\n", target(1), target(2), v, w);
    yaw = yaw + error_theta;
    
    pause(0.1); % tempo tra i comandi
end

% Stop finale
vel_msg.Linear.X = 0;
vel_msg.Angular.Z = 0;
send(vel_pub, vel_msg);

%% CHIUDERE ROS
rosshutdown