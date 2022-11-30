clear all 
close all

global d h k m delay;
% 'd' is the base-point distance from the centroid
% 'k' & 'm' are the control parameters
d = 2; k = 2; m = 5;

h = .08; % discretized step size

delay = 0; % communication delay


n_max = 0; % initial noise
iterations = 20; % total trials

% lane-change reference trajectory
position = importdata("lane_change.mat");

T = size(position,2)-1;
t = 1:T;

xVelocity = (position(1,t+1) - position(1,t))/h;
yVelocity = (position(2,t+1) - position(2,t))/h;

figure
for p = 1:4 % level loop
    yb = [];xb = [];linear_vel = [];angular_vel = [];
    linear_noise = [];angular_noise = [];nv = [];nw = [];
    n1_max = [];n2_max = [];v = [];w = [];
    error_norm_max = [];error_norm = [];
    theta = [];theta_hat = [];

    for j = 1:iterations % trail loop
        zc = [];
        zc(:,1) =[position(1,1)-d; position(2,1)+d*(-n_max/5 + (n_max/5 + n_max/5)*rand()); 0];
        xb(j,1) = zc(1,1) + d*cos(zc(3,1));
        yb(j,1) = zc(2,1) + d*sin(zc(3,1));
        
        theta(j,1) = zc(3,1); 
        theta_hat(j,1) = zc(3,1);

        for t = 1 : T
            % generate angular and linear noise
            nv(j,t) = -n_max/5 + (n_max/5 + n_max/5)*rand(); 
            nw(j,t) = -n_max + (n_max + n_max)*rand();
    
            if delay == 0 
                x = [xb(j,t); yb(j,t); theta(j,t)];   
            else
                x = [xb(j,t); yb(j,t); theta_hat(j,t)];
            end
            % controller returns the desired velocities
            [v(j,t), w(j,t)] = controller(x, xVelocity(t), yVelocity(t),position(:,t));
    
            linear_noise(j,t) = v(j,t)*nv(j,t);
            angular_noise(j,t) = w(j,t)*nw(j,t);
           
            % use the dynamics with desired velocities to generate the velocities
            dot_x = dynamics(theta(j,t),v(j,t),w(j,t),linear_noise(j,t),angular_noise(j,t));
             
            % centroid dynamics

            % integrate the position to get the new position
            xb(j,t+1) = xb(j,t) + dot_x(1)*h;
            yb(j,t+1) = yb(j,t) + dot_x(2)*h;
            theta(j,t+1) = theta(j,t) + dot_x(3)*h;
           
            zc(:,t+1) = [xb(j,t+1); yb(j,t+1); theta(j,t+1)] - [d*cos(theta(j,t+1)); d*sin(theta(j,t+1)); 0];

            if theta(j,t+1) > pi
                theta(j,t+1) = theta(j,t+1)-2*pi;
            elseif theta(j,t+1) < -pi
                    theta(j,t+1) = theta(j,t+1)+2*pi;
            end 
    
            w_delay = [];
            if t >= 1+delay
                i=0;
                for l = t-delay:t
                    i = i+1;
                    w_delay(i) = w(j,l);
                end
                if v(j,t-delay) ~= 0 
                    theta_hat(j,t+1) = atan2((zc(2,t+1-delay)-zc(2,t-delay))/(v(j,t-delay)), ...
                    (zc(1,t+1-delay)-zc(1,t-delay))/(v(j,t-delay))) + sum(w_delay)*h;
                else 
                    theta_hat(j,t+1) = atan2(zc(2,t+1-delay)-zc(2,t-delay), ...
                    zc(1,t+1-delay)-zc(1,t-delay))+ sum(w_delay)*h;
                end
            else
                i=0;
                for l = 1:t
                    i = i+1;
                    w_delay(i) = w(j,l);
                end

                theta_hat(j,t+1) = theta(j,1)+sum(w_delay)*h;
            end
        end
    end
    
    % error 
    error_norm_max = 0;theta_e_max = 0;error_int = 0;
    n2_max = 0;n1_max = 0;theta_e = [];
    for j = 1:iterations
        for i = 1:length(position)
            error = [position(1,i) - xb(j,i); position(2,i) - yb(j,i)];
            error_norm(i) = norm(error);
            theta_e(i) = abs(theta(j,i)-theta_hat(j,i));
        end
        if j > 1 
            if max(error_norm(1)) > error_int
                error_int = max(error_norm(1));
            end
            if max(angular_noise(j,:)) > n2_max 
                n2_max = max(angular_noise(j,:));
            end
            if max(linear_noise(j,:)) > n1_max 
                n1_max = max(linear_noise(j,:));
            end
    
            if max(error_norm(:)) > error_norm_max
               error_norm_max = max(error_norm(:));
            end
            if max(theta_e(i)) > theta_e_max
                theta_e_max = max(theta_e(:));
            end
        else 
            error_int = max(error_norm(1));
            n2_max = max(angular_noise(j,:));
            n1_max = max(linear_noise(j,:));
            error_norm_max = max(error_norm(:));
            theta_e_max = max(theta_e(i));
        end
    end

u = [];
% velocity norm & noise
velocity_norm(1) = 0;
for i = 1:length(position)-1
    Z = [xVelocity(i); yVelocity(i)];
    reference_velocity_norm(i) = norm(Z);
    u(i) = (n_max/5)^2+n_max^2*d^2;
end

% velocity bounds for control
for i = 1:length(position)-1
    max_v(i) = max(reference_velocity_norm)+k*min(m,error_norm_max);
    max_w(i) = 1/d*max(reference_velocity_norm)+k/d*min(m,error_norm_max);
end

% tracking error bounds
e_upper = [];
for t = 0:length(position)-1
    if error_norm(t+1) > m
        c1 = 2*k*cos(theta_e_max);
        a = .5*sqrt(c1);
        b = a;
    
        l = a^2+b^2;
        c2 = (2*max(reference_velocity_norm)^2/a^2)*max(sin(theta_e_max/2).^2)+1/b^2*max(u);
        c3 = c1*m^2-c2;
        
        e_upper(t+1) = ((error_int^2-c3/l)*exp(l*t*h)+c3/l)^(1/2);
    else 
        c1 = 2*k*cos(theta_e_max);
        a = .5*sqrt(c1);
        b = a;
        c2 = (max(reference_velocity_norm)^2/a^2)*max(1-cos(theta_e_max))+1/b^2*max(u);
        l = a^2+b^2;

        e_upper(t+1) = sqrt(exp(-(c1-l)*t*h)*error_norm(1)^2+c2/(c1-l)*(1-exp(-(c1-l)*t*h)));
        e_const(t+1) = sqrt(c2/(c1-l));
    end
end

subplot(2,2,p)

if isreal(e_upper) ~= 1
    e_upper = [];
else
    % plot the tracking bounds
    for i = 1:1:length(position)
        circle(position(1,i), position(2,i),e_upper(i),[.7 .7 .7],1);
    end
end

hold on
h1(1) = plot(position(1,:),position(2,:),'color','b','LineWidth',2,'DisplayName','reference trajectory');

for j = 1:iterations
    h1(2) = plot(xb(j,:), yb(j,:),'color',[1 0 0],'DisplayName',sprintf('base-point for %delay trails',iterations));
end

axis equal
xlim([d 45])
ylim([-10 5])
title({['$\delta$ = ', num2str(delay), ', $\bar{n}_v = v_{\max}[-$', num2str(n_max/2),', ', num2str(n_max/2),'$]$, ', ...
    '$\bar{n}_w = w_{\max}[-$', num2str(n_max),', ', num2str(n_max),'$]$']},'Interpreter','latex')
daspect([1 1 1])
set(gca,'xticklabel',[])
set(gca,'yticklabel',[])
hold off

% increase delay and noise level
delay = delay+4;
n_max = n_max+.5;
end

function [v, w] = controller(x, xVelocity, yVelocity,position)
global d k m

J = [cos(x(3)) -d*sin(x(3));
    sin(x(3))  d*cos(x(3)) ];
E = x(1:2,1)- [position(1); position(2)];

if norm(E) > m
        v_feedback = [xVelocity; yVelocity] - k*m/norm(E)*E;
    else
    v_feedback = [xVelocity; yVelocity] - k*E;
end

desired_control = (J)\v_feedback;

v = desired_control(1);
w = desired_control(2);

end

function dot_x = dynamics(theta,v,w,nv,nw)
global d
M = [cos(theta) -d*sin(theta);
     sin(theta)  d*cos(theta);
     0               1      ];

dot_x = M*[v+nv; w+nw];
end

function circles = circle(x,y,r,c,trans)
hold on
th = 0:pi/50:2*pi;
x_circle = r * cos(th) + x;
y_circle = r * sin(th) + y;
fill(x_circle, y_circle,c,'LineStyle','none','FaceAlpha', trans,'HandleVisibility','off')
end
