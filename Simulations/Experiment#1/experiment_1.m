clear all 
close all

global d h k m
% 'd' is the base-point distance from the centroid
% 'k' & 'm' are the control parameters
d = .3; k = 1.3; m = .3;

h = .02; % discretized step size

% get circular reference trajectory
position = importdata("circle.mat");

% down_sample is used to increase the reference velocity
% default value is 1
down_sample_amount = 1; 

l = 1;
for i = 1:down_sample_amount:length(position)
    new_position(:,l) = position(:,i);
    l = l+1;
end
position = new_position;

%%%%%%%%%%%%%%%%%%%% initialize states %%%%%%%%%%%%%%%%%%%%%%
% centroid position and heading angle
zc(:,1) =[position(1,1)+d*2; position(2,1); pi/4]; 
xb(1) = zc(1,1) + d*cos(zc(3,1)); % x base-point position
yb(1) = zc(2,1) + d*sin(zc(3,1)); % y base-point position
theta(1) = zc(3,1); % heading angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% total iterations
T = size(position,2)-1;
t = 1:T;

% reference velocity
xVelocity = (position(1,t+1) - position(1,t))/h;
yVelocity = (position(2,t+1) - position(2,t))/h;

% control loop
for t = 1 : T
     % current state
     x = [xb(t); yb(t); theta(t)];
    
    % controller returns the desired velocities
    [v, w] = controller(x, xVelocity(t), yVelocity(t),position(:,t));

    % use the dynamics with desired velocities to generate the velocities
    dot_x = dynamics(theta(t), v, w);
    
    % velocity history
    linear_vel(t) = v;
    angular_vel(t) = w;    
        
    % base-point position
    xb(t+1) = xb(t) + dot_x(1)*h;
    yb(t+1) = yb(t) + dot_x(2)*h;
    theta(t+1) = theta(t) + dot_x(3)*h;
   
    % Centroid position
    zc(:,t+1) = zc(:,t)+ [cos(theta(t)) 0; sin(theta(t)) 0; 0 1]*[linear_vel(t); angular_vel(t)]*h;
end

% tracking error
for i = 1:length(position)
    error = [position(1,i) - xb(i); position(2,i) - yb(i)];
    error_norm(i) = norm(error);
end

% velocity norm
velocity_norm(1) = 0;
for i = 1:length(xb)-1
    Z = [xVelocity(i); yVelocity(i)];
    reference_velocity_norm(i) = norm(Z);
end

% generate velocity bounds
for i = 1:length(xb)-1
    max_v(i) = max(reference_velocity_norm)+k*min(m,error_norm(i));
    max_w(i) = 1/d*max(reference_velocity_norm)+k/d*min(m,error_norm(i));
end

figure
title("Trajectory")
plot(position(1,:),position(2,:),'.-','color','b'); hold on 
plot(zc(1,:), zc(2,:),'color', 'g'); 
plot(xb, yb,'color','r')
legend('reference trajectory','centroid', 'base-point',  "location", ...
    'southeast','Interpreter','Latex', 'Fontsize',12)
axis equal
hold off

%bounded velocities
figure
title("Velocity")
subplot(2,1,1)
plot([1:length(xb(1,:))-1]*h, max_v)
hold on
plot([1:length(xb(1,:))-1]*h, abs(linear_vel))
legend('$v_{upper}$','$|v|$','location','northeast','Interpreter','Latex', 'Fontsize',12)
hold off

subplot(2,1,2)
plot([1:length(xb(1,:))-1]*h, max_w)
hold on
plot([1:length(xb(1,:))-1]*h, abs(angular_vel))
legend('$w_{upper}$','$|w|$','location','northeast','Interpreter','Latex', 'Fontsize',12)
xlabel('t','Interpreter','latex', 'Fontsize',14)
hold off

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

function dot_x = dynamics(theta,v,w)
global d

M = [cos(theta) -d*sin(theta);
     sin(theta)  d*cos(theta);
     0               1      ];

dot_x = M*[v; w];

end
