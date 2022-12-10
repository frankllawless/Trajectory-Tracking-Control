clear all 
close all

global h d epsilon p
% 'd' is the distance of the pen from the base of the robot
d = .3;
p = .25; % adjust to slow down velocities/increase convergence time
epsilon = .2; % desired tracking performance

h = .005; % step size

% noise magnitudes
angular_noise = .1; % max 1
linear_noise = .1; % max 1

%make reference circle
th = 0:h:2*pi;
for i = 1:length(th)
    xunit = 1 * cos(th(i)) + 0;
    yunit = 1 * sin(th(i)) + 1;
    position(:,i) = [xunit; yunit];
end

% Initialize position
theta_int = pi/6;
state(:,1) =[position(1,1)-d*cos(theta_int)+1; position(2,1)-d*sin(theta_int); theta_int];
xb(1) = state(1,1) + d*cos(state(3,1));
yb(1) = state(2,1) + d*sin(state(3,1));

xba = xb(1);
yba = yb(1);

theta_hat(1) = state(3,1); % estimated theta
thetatk = theta_hat(1); % theta at triggering time
thetaa = theta_hat(1); % actual theta
T = size(position,2)-1;
t = 1:T;

% Get reference velocities
xVelocity = (position(1,t+1) - position(1,t))/h;
yVelocity = (position(2,t+1) - position(2,t))/h;

% Calculated maximum reference velocities
vr = max(sqrt(xVelocity.^2+yVelocity.^2));
wr = vr/d;

% Bound noise
nv = linear_noise*vr;
nw = angular_noise*vr/d;
eta1 = sqrt(nv^2+d^2*nw^2);

% Initial error
e(:,1) = [position(1,1)-xb(1); position(2,1)-yb(1)];
e_tk = e(:,1);


tk = 0;
o1 = eta1/vr*(1+eta1/(nw*norm(e(:,1))));
o2 = eta1^2/(vr*nw*norm(e(:,1)));

pk = (-o1+sqrt(o1^2+4*o2))/2 +p; % p is used to slightly adjust pk 

tf = tk + pk*norm(e_tk)/eta1;

% Initial equation 3.15
b1 = norm(e_tk)/(tf-tk)-eta1;
b2 = (vr+norm(e_tk)/(tf-tk))*nw/2;

tk1 = tk+b1/(2*b2);

CT = eta1^2*(1-pk)^2/(2*nw*(vr*pk^2+eta1*pk));
N = ceil((norm(e(:,1))-epsilon)/CT);

actual_error(:,1) = e(:,1);
error_history(1) = norm(e(:,1));
w_history = [];
i = 0; k = 1; first = true; trigger_time = 1; trigger = false;
for t = [0 : 1 : T-1]*h %Trigger t = tk1
    i = i+1;

    if t <= tf
        e_th(i) = norm(e_tk) - b1*(t-tk) + b2*(t-tk)^2;
    else
%         keyboard;
        e_th(i) = eta1*(t-tk) + b2*(t-tk)^2;
    end


    if t > tk1 -h
        w_history = [];
        trigger_time = 1;
        k = k+1;
        tk = t;
        e_tk = actual_error(:,i);

        o1 = eta1/vr*(1+eta1/(nw*norm(actual_error(:,i))));
        o2 = eta1^2/(vr*nw*norm(actual_error(:,i)));

        pk = (-o1+sqrt(o1^2+4*o2))/2 +p;
        if pk >= .99
            pk = .99;
        end
        tf = tk + pk*norm(e_tk)/eta1;
        
        b1 = norm(e_tk)/(tf-tk)-eta1;
        b2 = (vr+norm(e_tk)/(tf-tk))*nw/2;

        if norm(e_tk) > epsilon && first == true
            tk1 = tk+b1/(2*b2);
        else
            first = false;
            tk1 = tk + (b1+sqrt(b1^2-4*b2*(norm(e_tk)-epsilon)))/(2*b2);
            if tk1 > tf 
               tk1 = max(tk + (-eta1 + sqrt(eta1^2 + 4 * epsilon* b2) )/(2*b2), tf);
               if tk + (-eta1 + sqrt(eta1^2 + 4 * epsilon* b2) )/(2*b2) < tf
                   keyboard
               end
            end
        end
        trigger = true;
    end

    if trigger == true % Save tk/e_th when there is a trigger
        trigger = false;
        trigger_history(k) = tk;
        error_history(k) = e_th(i);
        trigger_position(:,k) = [xba(i); yba(i)];
        ref_position(:,k) = [position(1,i); position(2,i)];

        error_lineszb(:,k) = [xba(i), yba(i)];
        error_lineszr(:,k) = [position(1,i), position(2,i)];
        % estimates are updated
        xb(i) = xba(i);
        yb(i) = yba(i);
        theta_hat(i) = thetaa(i);

    end

    x = [xb(i); yb(i); theta_hat(i)];

    [v(i), w(i)] = controller(x, e_tk, t, tf, tk, xVelocity(i), yVelocity(i));

    n1(i) = (linear_noise-2*linear_noise*rand)*vr;
    n2(i) = (angular_noise-2*angular_noise*rand)*wr;

    linear_vel(i) = v(i)+n1(i);
    angular_vel(i) = w(i)+n2(i);

    % Propagate the estimated dynamics
    dot_x(:,i) = estimated_dynamics(theta_hat(i),v(i),w(i));
    xb(i+1) = xb(i) + dot_x(1,i)*h;
    yb(i+1) = yb(i) + dot_x(2,i)*h;
    theta_hat(i+1) = theta_hat(i) + dot_x(3,i)*h;

    % Propagate the actual dynamics
    dota_x(:,i) = dynamics(thetaa(i),v(i),w(i),n1(i), n2(i));
    xba(i+1) = xba(i) + dota_x(1,i)*h;
    yba(i+1) = yba(i) + dota_x(2,i)*h;
    thetaa(i+1) = thetaa(i) + dota_x(3,i)*h;

    %save error and centroid
    e_estimated(:,i+1) = position(:,i+1) - [xb(i+1); yb(i+1)];
    actual_error(:,i+1) = position(:,i+1) - [xba(i+1); yba(i+1)];
    state(:,i+1) = [xba(i+1)-d*cos(thetaa(i+1));yba(i+1)-d*sin(thetaa(i+1));thetaa(i+1)];
end

for i = 1:length(trigger_history)
    trigger_x(i) = trigger_history(i);
    trigger_y(i) = [error_history(i)];
end
for i = 1:length(actual_error)
    error_norm(i) = norm(actual_error(:,i));
end


figure
hold on
plot(position(1,:),position(2,:),'.-','color','b');
plot(state(1,:), state(2,:),'color', 'g');
plot(xba, yba,'color','r')
legend('reference trajectory','centroid','base-point', ...
    "location", 'southeast','Interpreter','latex', FontSize=12)
axis equal
title("Trajectory",'Interpreter','latex', FontSize=14)
hold off


figure
fig1(1) = plot([0:size(xb,2)-1]*h, error_norm,'Color','r','DisplayName','$\|e\|$','LineWidth',1.2);
hold on
fig1(3) = plot([0, size(xb,2)-1]*h, [1,1]*epsilon,'DisplayName','$\epsilon$');
fig1(2) = plot([0:size(xb,2)-2]*h, e_th,'Color','b','DisplayName','$e_{\rm{upper}}$','LineWidth',1.2);
title("Error Performance",'FontSize',12,'Interpreter','Latex')
scatter(trigger_x, trigger_y,'filled','MarkerFaceColor','m', ...
    'MarkerEdgeColor','k')
grid on

legend(fig1([1 2 3]),'location','northeast','Interpreter','latex',FontSize=12)
ylabel('$\|e(t)\|$','Interpreter','latex',FontSize=14)
xlabel('$t$','Interpreter','latex',FontSize=14)
xlim([0, (size(xb,2)-2)*h])
hold off

figure
plot([1:size(v,2)]*h, abs(v)); hold on
plot([1:size(w,2)]*h, abs(w));
legend('|v|', '|w|')
title("Velocities",'Interpreter','latex')
hold off

figure
plot([0:size(xb,2)-1]*h, theta_hat)
hold on
plot([0:size(xb,2)-1]*h, thetaa)
legend("Estimated", "Actual",'Interpreter','latex')
title("Heading angle error",'Interpreter','latex')
hold off

function [v, w] = controller(x, e_tk, t, tf, tk, xVelocity, yVelocity)
global d epsilon
J = [cos(x(3)) -d*sin(x(3));
    sin(x(3))  d*cos(x(3)) ];

if t < tf
    u = e_tk/(tf-tk);
else
    u = 0;
end

v_feedback = [xVelocity; yVelocity] + u;

desired_control = (J)\v_feedback;

v = desired_control(1);
w = desired_control(2);

end

function dot_x = estimated_dynamics(theta,v,w)
global d

M = [cos(theta) -d*sin(theta);
    sin(theta)  d*cos(theta);
    0               1      ];

dot_x = M*[v; w];
end

function dot_x = dynamics(theta,v,w,n1,n2)
global d

M = [cos(theta) -d*sin(theta);
    sin(theta)  d*cos(theta);
    0               1      ];

dot_x = M*[v+n1; w+n2];
end
