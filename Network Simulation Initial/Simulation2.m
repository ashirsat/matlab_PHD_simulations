clc;
clear all;
close all;
%% Initializing the params
% Time
t = linspace(0,100)';
% Number of robots
n = 10;
% Initial poses
x0 = -n + (2*n)*randn(n,1);
y0 = -n + (2*n)*randn(n,1);
% Initial heading
theta0 = (2*pi/180)*(-n + (2*n)*randn(n,1));

% Inital velocities
v_mag = 1;
vx0 = v_mag*cos(theta0);
vy0 = v_mag*sin(theta0);

% Setting the perimeter
perimeter_limit = 100;

x_next = zeros(n,1);
y_next = zeros(n,1);

%% Calculating the velocity trajectories
disp_x = zeros(length(t),n);
disp_y = zeros(length(t),n);
time = 1;
v_const = 1;

while time < length(t)
    % Creating a random switching variable
    switch_rand = rand(1);
    % randomizing Heading
    alpha = 2*pi*(-n + ((2*n)*rand(1)))/n;
    % Randomizing Velocity magnitude
    beta = (-n + ((2*n)*rand(1)))/n;
    for i =1:n
        delx(time,i) = vx0(i)*(t(time+1) - t(time));
        dely(time,i) = vy0(i)*(t(time+1) - t(time));
        x_next(time,i) = x0(i) + delx(time,i);
        y_next(time,i) = y0(i) + dely(time,i);
        x0(i) = x_next(time,i);
        y0(i) = y_next(time,i);
            
    end
    % Creating a perimeter for exploration
%     if (norm(x_next(time,:)) > perimeter_limit) || ...
%             (norm(y_next(time,:)) > perimeter_limit)
%         vx0 = vx0.*cos(theta0);
%         vy0 = -vy0.*sin(theta0);
%     else
        if switch_rand > 0.8 
            % Causing a random change in heading and magnitude of velocity
            theta0 = theta0 + (alpha)*(-n + (2*n)*rand(n,1))*(2*pi);
            vx0 = vx0 + v_const*beta*cos(theta0);
            vy0 = vy0 + v_const*beta*sin(theta0);
        else
            % Causing the complimentary change in the heading 
            theta0 = theta0 + (1- alpha)*(-n + (2*n)*rand(n,1))*(2*pi);
            vx0 = vx0 + v_const*(1-beta)*cos(theta0);
            vy0 = vy0 + v_const*(1-beta)*sin(theta0);
        end
%     end
    
    time = time +1;
end
figure(1)
p1 =plot(x_next, y_next);
hold on
line([-perimeter_limit perimeter_limit], [perimeter_limit, perimeter_limit], 'LineWidth',5);
line([-perimeter_limit perimeter_limit], [-perimeter_limit -perimeter_limit], 'LineWidth',5);
line([perimeter_limit perimeter_limit], [-perimeter_limit perimeter_limit], 'LineWidth',5);
line([-perimeter_limit -perimeter_limit], [-perimeter_limit perimeter_limit],'LineWidth',5);
l1 = legend('${Robot_1}$','${Robot_2}$','${Robot_3}$','${Robot_4}$','${Robot_5}$','${Robot_6}$','${Robot_7}$','${Robot_8}$','${Robot_9}$','${Robot_10}$');
set(l1,'Interpreter','latex');
grid on;

%% Creating the adjacency matrix
% A = zeros(n,n,length(t));
time =1;
while time < length(t)
    for i =1:n
        for j=1:n
            if (norm(delx(time,i) - delx(time,j)) < 100) && (norm(dely(time,i) - dely(time,j)) < 100)
                if i==j
                    A(i,j,time) = 0;
                else
                    A(i,j,time) = 1;
                end
            else
                A(i,j,time) =0;
            end
        end
    end
    time = time+1;
end

