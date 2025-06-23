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
img = imread('cropped_maze.pgm');
map = img < 250;
resolution = 20; % 1/0.05 metri per cella
% plot di sicurezza
figure;

% imagesc(map);
% colormap(gray);
axis equal tight;

%%
mapSize = size(map);
odom_sub = rossubscriber('/odom', 'nav_msgs/Odometry');
odom_msg = receive(odom_sub, 3);
position = odom_msg.Pose.Pose.Position;
ori = odom_msg.Pose.Pose.Orientation;
[x_start, y_start] = coordToCell(position.X, position.Y, map);
% start = [x_start, y_start];
start = [20, 20];
goal = [187, 70];

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
map_resolution = 0.05;  % metri per cella
K_v = 0.5;              % guadagno per velocità lineare
K_w = 1.5;              % guadagno per velocità angolare
v_max = 2.2;            % velocità lineare massima
w_max = 1.8;            % velocità angolare massima
goal_tolerance = 0.01;   % distanza minima al target
angle_tolerance = deg2rad(10); % tolleranza angolare (~10°)
% clean_path = [190, 100; 130, 100]; %%% PROVVISORIO %%%
% Publisher comandi velocità
vel_pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
vel_msg = rosmessage(vel_pub);

% Subscriber alla posizione del robot
odom_sub = rossubscriber('/odom', 'nav_msgs/Odometry');

% % Converte path da celle a metri
% clean_path_m = (clean_path * map_resolution);

% Loop sui punti del path
for i = 1:(size(clean_path, 1)-1)
    target = clean_path(i+1, :)';  % punto target [x; y]
    reached = false;

    while ~reached
        % Leggi odometria
        odom_msg = receive(odom_sub, 1);
        pos = odom_msg.Pose.Pose.Position;
        ori = odom_msg.Pose.Pose.Orientation;

        % Estrai stato attuale
        x = pos.X;
        y = pos.Y;
        [x_current, y_current] = coordToCell(x, y, map);
        quat = [ori.W, ori.X, ori.Y, ori.Z];
        eul = quat2eul(quat);
        yaw = eul(1);
        current_pose = [x_current; y_current; yaw];

        % Calcola distanza e angolo
        delta = target(1:2) - current_pose(1:2);
        rho = norm(delta)/map_resolution;
        theta_target = atan2(delta(2), delta(1));
        error_theta = wrapToPi(theta_target - yaw);

        % Controllo rotazione prima di avanzare
        v = K_v * rho;
        w = K_w * error_theta;

        % Limiti di velocità
        v = min(max(v, -v_max), v_max);
        w = min(max(w, -w_max), w_max);

        % Costruisci e invia il messaggio
        vel_msg.Linear.X = v;
        vel_msg.Angular.Z = w;
        send(vel_pub, vel_msg);

        % Debug
        fprintf("→ Target: [%.2f %.2f] | rho = %.2f | θ err = %.1f° | v = %.2f | w = %.2f\n", ...
                target(1), target(2), rho, rad2deg(error_theta), v, w);

        % Criterio di arrivo
        if rho < goal_tolerance && abs(error_theta) < angle_tolerance
            reached = true;
        end

        pause(0.1);
    end
end

% Stop finale
vel_msg.Linear.X = 0;
vel_msg.Angular.Z = 0;
send(vel_pub, vel_msg);

%% CHIUDERE ROS
rosshutdown