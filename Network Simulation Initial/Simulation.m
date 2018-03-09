clc;
clear all;
close all;
import robot;
 %% Initialising the robots and various parameters
n = 5;
t = transpose(linspace(0,1000,100));
%% Initial orientation
theta_0 = 2*pi*(-n + (2*n)*rand(n,1));

data = 1/(n);
v_const = 1;
vx_0 = v_const*cos(theta_0);
vy_0 = v_const*sin(theta_0);

%% Initial Positions
x_0 = -n + (2*n)*rand(n,1);
y_0 = -n + (2*n)*rand(n,1);

%%
for i =1:n
    r(:,i) = robot(i,x_0(i),y_0(i),theta_0(i),data); 
end
i =1;
vx_ini = vx_0;
vy_ini = vy_0;
t_curr = t(1);
% del_x = zeros(length(t),n);
% del_y = zeros(length(t),n);
t_next = t_curr;
% disp_x = zeros(length(t),n);
% disp_y = zeros(length(t),n);
% rand_switch = transpose(rand(1,100));
while t_curr < 100
    rand_switch = rand(1);
    alpha = 2*pi*(-n + ((2*n)*rand(1)))/n;
    beta = (-n + ((2*n)*rand(1)))/n;
    for j =i:n-1
        del_x(i,j) = vx_ini(j)*(t_next - t_curr);
        del_y(i,j) = vy_ini(j)*(t_next - t_curr);
        disp_x(i,j) = x_0(j) + del_x(i,j);
        disp_y(i,j) = y_0(j) + del_y(i,j);
        x_0(j) = disp_x(i,j);
        y_0(j) = disp_y(i,j);           
        t_curr;
    end
%     r(i).getodometry(disp_x(:,i),disp_y(:,i),theta_0(:,i));    
%     if i>50
    if rand_switch  > 0.8 
%         beta = (-n + ((2*n)*rand(1)))/n;
        theta_0 = theta_0 + (alpha)*(-n + (2*n)*rand(n,1))*(2*pi);
        vx_new = vx_ini +beta*(v_const*cos(theta_0));
        vy_new = vy_ini +beta*(v_const*sin(theta_0));
        vx_ini = vx_new;
        vy_ini = vy_new;
    else
        theta_0 = theta_0 + (1-alpha)*(-n + (2*n)*rand(n,1))*(2*pi);
        vx_new = vx_ini +(1-beta)*(v_const*cos(theta_0));
        vy_new = vy_ini +(1-beta)*(v_const*sin(theta_0));
        vx_ini = vx_new;
        vy_ini = vy_new;
    end
    if i < 100
        t_curr = t(i);
        t_next = t(i+1);
    else
        break;
    end
    theta_del(:,i)= theta_0;
    i = i+1; 
end
theta_new = [theta_0';
            theta_del'];
%     
figure(1);
p1 = plot(disp_x,disp_y);
l1 = legend('${Robot_1}$','${Robot_2}$','${Robot_3}$','${Robot_4}$','${Robot_5}$','${Robot_6}$','${Robot_7}$','${Robot_8}$','${Robot_9}$','${Robot_10}$');
set(l1,'Interpreter','latex');
grid on;
%}            
%{
figure(2);
plot(theta_0*180*pi);
hold on;
grid on;
plot(theta_new*180/pi);
legend('{\theta_0}','{\theta_{\delta}}');
xlabel('Time');
ylabel('{\theta}');
%}
%% Assigning the values to the robot class 
i=1;
for i=1:n
  rob(i) = r(:,i).getodometry(disp_x(:,i), disp_y(:,i), theta_new(:,i));
end
rob = rob';
%% Calculating the distance to the neighbours
l =1;
i=1;
for i=1:n    
    if i <= n-1 %% Go though the robots from i to n-1
        for j =i+1:n-1
            for time =1:length(t)
                xdist(i,j,time) = norm((rob(i).X(time) - rob(j).X(time)));
                ydist(i,j,time) = norm((rob(i).Y(time) - rob(j).Y(time)));
                l=l+1;
                if xdist(i,j,time) < 100 && ydist(i,j,time) < 100
                    A(i,j,time) = 1;
                    A(j,i,time) = 1;
                else
                    A(i,j,time) = 0;
                    A(j,i,time) = 0;
                end
%                 lambda(:,:,time) = eig(A(:,:,time));
            end
        end
    else
        for j =n-1:n %% For the final iteration from robot n to n
            for time =1:length(t)
                xdist(i,j,time) = norm((rob(i).X(time) - rob(j).X(time)));
                ydist(i,j,time) = norm((rob(i).Y(time) - rob(j).Y(time)));
                l=l+1;
%                 if xdist(i,j,time) < 100 & ydist(i,j,time) < 100
%                     A(i,j,time) = 1;
%                     A(j,i,time) = 1;
%                 else
%                     A(i,j,time) = 0;
%                     A(j,i,time) = 0;
%                 end
            end
            
        end
    end
%}
end

%{
%     if ii==5
%         print(ii) 
%     end
%     avgx(time,:) = mean(xdist(time,:,:));
%     avgy(time,:) = mean(ydist(time,:,:));
%     varx(time,:) = var(xdist(time,:,:));
%     vary(time,:) = var(ydist(time,:,:));
%}

l=1;
%% Calculating the distances between robots for all tiem
for i=1:n
%     for j=i:n
%         for time=1:length(t)
%             xdist2(i,j,time) = norm((rob(i).X(time) - rob(j).X(time)));
%             ydist2(i,j,time) = norm((rob(i).Y(time) - rob(j).Y(time)));
%             l=l+1;
%         end
%     end
    avgx_t(i,:) = mean(xdist(i,:,:));
    avgy_t(i,:) = mean(ydist(i,:,:));
    varx_t(i,:) = var(xdist(i,:,:));
    vary_t(i,:) = var(ydist(i,:,:));
end
%}

%{
figure(3)
plot(xdist(:,1,2),ydist(:,1,2),'-b');
hold on
grid on
plot(xdist(:,1,3),ydist(:,1,3),'-r');
plot(xdist(:,1,4),ydist(:,1,4),'-g');
plot(xdist(:,1,5),ydist(:,1,5),'-k');
plot(xdist(:,1,6),ydist(:,1,6),'-k');
plot(xdist(:,1,7),ydist(:,1,7),'--b');
plot(xdist(:,1,8),ydist(:,1,8),'--r');
plot(xdist(:,1,9),ydist(:,1,9),'--g');
plot(xdist(:,1,10),ydist(:,1,10),'--k');
% plot(avgx(:,1),mean(ydist),'--r');
legend('Robot1-2','Robot1-3','Robot1-4','Robot1-5','Robot1-6','Robot1-7','Robot1-8','Robot1-9','Robot1-10');
x2 =xlabel('$\|{X}\|$');
y2=ylabel('$\|{Y}\|$');
set(x2,'Interpreter','latex');
set(y2,'Interpreter','latex');
title('Relateive euclidean distance for robot1')
 %}

%% comunicating the data between robots
%%% if the robots are within certain circle of each other 
for time =1:length(t)
    for i=1:n 
        for j= i+1:n
            if norm(avgx_t(i,time)- avgx_t(j)) < 100 & norm(avgy_t(i,time)- avgy_t(j,time)) < 100
                r(i).sum = (r(j).data + r(i).data);
                r(j).sum = (r(i).data + r(j).data);
                rob2(i,j,:) = [r(i).data,r(j).data];
            else
                continue;
            end
        end
    end
end
%{
figure(4)
plot(rob2);
%}
                
