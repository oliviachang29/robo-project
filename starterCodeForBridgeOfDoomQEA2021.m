function starterCodeForBridgeOfDoomQEA2021()
% Insert any setup code you want to run here
clear;
% % u will be our parameter
% syms u;
% 
% % this is the equation of the bridge
% R = 4*[0.396*cos(2.65*(u+1.4));...
%        -0.99*sin(u+1.4);...
%        0];
% 
% % tangent vector
% T = diff(R);
% 
% % normalized tangent vector
% That = T/norm(T);

% define variables
syms u

% beta * t = u
% lower beta = lower speed
beta = 0.3;
timeBounds = [0 (3.2/beta)];

% Define equations
ri=4*0.3960*cos(2.65*(beta*u + 1.4));
rj=-4*0.99*sin(beta*u+1.4);
rk=0*u;
r=[ri,rj,rk];

% Assume u is positive
assume(u,{'real','positive'})

% ---- FIND UNIT TANGENT VECTOR ----
dr=diff(r,u);
T_hat=simplify(dr./norm(dr))

% ---- FIND UNIT NORMAL VECTOR----
dT_hat=diff(T_hat,u);
N_hat=dT_hat/norm(dT_hat);
N_hat=simplify(N_hat)

% ---- FIND VELOCITY ----
v = simplify(dr)
speed = simplify(norm(v))

% ---- FIND ANGULAR VELOCITY ----
angular_velocity = simplify(cross(T_hat,dT_hat))

% ---- FIND WHEELS ----
% since angular velocity is always in k hat directly only,
% multiplying norm(angular_velocity) with sign(angular_velocity)
% is the same as the dot product of angular velocity with k hat
d = 0.235;
v_left_wheel = simplify(speed - (norm(angular_velocity) * sign(angular_velocity) * d)/2)
v_right_wheel = simplify(speed + (norm(angular_velocity) * sign(angular_velocity) * d)/2)

% ---- FIND CURVE DISTANCE ----
distance = 3; %arbitrary for now

% stop the robot if it's going right now
stop;

bridgeStart = double(subs(r,u,0));
startingThat = double(subs(T_hat,u,0));
placeNeato(bridgeStart(1),  bridgeStart(2), startingThat(1), startingThat(2));

% wait a bit for robot to fall onto the bridge
pause(2);

% start tracking time
rostic;
% time to drive!!
while true
    % get current time
    % timeBounds(1) is the beginning time
    u_val = rostoc() + timeBounds(1);
    
    % timeBounds 2 is the end time
    if u_val > timeBounds (2)
        break
    end
    
    leftSpeed = double(subs(v_left_wheel, u, u_val));
    rightSpeed = double(subs(v_right_wheel, u, u_val));
    
    drive(timeBounds, leftSpeed, rightSpeed);
end
stop;
end
