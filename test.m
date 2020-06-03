clear variables, close all
clc

rosshutdown;
rosinit('http://10.128.0.216:11311');

control_pub = rospublisher('/cmd_vel');
velmsg = rosmessage(control_pub);
joint_sub = rossubscriber('/joint_states');
imu_sub = rossubscriber('/imu');
pause(2)

total_samples = 500;

xhat_graph = zeros(total_samples,3);

xhat_prev = [0,0,0].';
Q_prev = [0.05 0 0; 0 0.05 0; 0 0 0.05];
R = [0.05 0 0; 0 0.05 0; 0 0 0.05];
xbar = [0, 0, 0].';
y = [0, 0, 0].';
F_prev = zeros(3);
P_poster_prev = eye(3);
K = zeros(3);
H = eye(3);

dT = 0.038; % Update value based on averaging of sample time.

velmsg.Angular.Z = 0.1;
velmsg.Linear.X = 0.1;
send(control_pub,velmsg);

for i = 1:total_samples
    joint = receive(joint_sub);
    
    xbar(1) = xhat_prev(1) + dT * velmsg.Linear.X * cos(xhat_prev(3));
    xbar(2) = xhat_prev(2) + dT * velmsg.Linear.X * sin(xhat_prev(3));
    xbar(3) = xhat_prev(3) + dT * velmsg.Angular.Z;
    
    F_prev = [1 0 -sin(xbar(3))*dT*velmsg.Linear.X;
        0 1 cos(xbar(3))*dT*velmsg.Linear.X;
        0 0 1];
    
    H = [1 0 -sin(xbar(3))*dT*(joint.Velocity(1)+joint.Velocity(2))/0.066;
        0 1 cos(xbar(3))*dT*(joint.Velocity(1)+joint.Velocity(2))/0.066;
        0 0 1];
    
    P_priori = F_prev * P_poster_prev * F_prev' + Q_prev;
    
    K = P_priori * H' * inv(H * P_priori * H' + R);
    
    y(1) = xhat_prev(1) + dT * (joint.Velocity(1)+joint.Velocity(2))/0.066 * cos(xhat_prev(3));
    y(2) = xhat_prev(2) + dT * (joint.Velocity(1)+joint.Velocity(2))/0.066 * sin(xhat_prev(3));
    y(3) = xhat_prev(3) + dT * (joint.Velocity(2) - joint.Velocity(1))/(0.287*0.033);
    
    xbar = xbar + K*(y - xbar);
    
    xhat_graph(i,:) = y;
    
    P_poster_prev = (eye(3) - K*H)*P_priori*(eye(3)-K*H)'+K*R*K';
end
velmsg.Angular.Z = 0.0;
velmsg.Linear.X = 0.0;
send(control_pub,velmsg);
rosshutdown;

figure(1)
hold on
plot(1:total_samples, xhat_graph(:,1))
grid on
hold off

figure(2)
hold on
plot(1:total_samples, xhat_graph(:,2))
grid on
hold off

figure(3)
hold on
plot(1:total_samples, xhat_graph(:,3))
grid on
hold off