clear variables, close all
clc

rosshutdown;
rosinit('http://10.128.0.216:11311');

control_pub = rospublisher('/cmd_vel');
velmsg = rosmessage(control_pub);
joint_sub = rossubscriber('/joint_states');
imu_sub = rossubscriber('/imu');
odom_sub =  rossubscriber('/odom');
scan_sub = rossubscriber('/scan');
pause(2)

total_samples = 300;

xhat_graph = zeros(total_samples,3);
time_stamp = zeros(total_samples,3);
odom_graph = zeros(total_samples,3);
imu_graph = zeros(total_samples,2);

b = 0.287;  % wheelbase of wafflepi.
r = 0.033;
time_prev = 0;
dT = 0.0753; % Update rate based on averaging of sample time. 0.038 = realbot

deltas = 0;
deltaTheta = 0;
xhat_priori = [0;0;0];
xhat_post_prev = [0; 0; 0;]; %Initial position, [x, y, theta]
F_prev = zeros(3);
Q_prev = zeros(3);
k = 0.0005;
P_priori = zeros(3);
P_post_prev = ([0.15; 0.15; 0.15] - xhat_post_prev)*transpose([0.15; 0.15; 0.15] - xhat_post_prev);
v_prev = 0;
v = 0;
H = zeros(3,3);
R = zeros(3);
K = zeros(3,3);
y = [0;0;0];
xhat_post = [0;0;0];

velmsg.Angular.Z = 0;
velmsg.Linear.X = 0.2;
send(control_pub,velmsg);

for i = 1:total_samples
    if i == 500
        velmsg.Angular.Z = 0;
        velmsg.Linear.X = 0.02;
        send(control_pub,velmsg);
    elseif i == 100
        velmsg.Angular.Z = 0.3;
        velmsg.Linear.X = 0.1;
        send(control_pub,velmsg);
    end
    
    xhat_priori(1) = xhat_post_prev(1) + dT*velmsg.Linear.X*cos(xhat_post_prev(3));
    xhat_priori(2) = xhat_post_prev(2) + dT*velmsg.Linear.X*sin(xhat_post_prev(3));
    xhat_priori(3) = xhat_post_prev(3) + dT*velmsg.Angular.Z;
    
    F_prev = [1, 0, -dT*velmsg.Linear.X*sin(xhat_post_prev(3));
        0, 1, dT*velmsg.Linear.X*cos(xhat_post_prev(3));
        0, 0, 1];
    
    joint = receive(joint_sub);
    imu = receive(imu_sub);
    odom = receive(odom_sub);
    
    imu_graph(i,1) = imu.LinearAcceleration.X;
    imu_graph(i,2) = imu.AngularVelocity.Z;
    
    tic

    odom_graph(i,1) = odom.Pose.Pose.Position.X;
    odom_graph(i,2) = odom.Pose.Pose.Position.Y;
    q0 = odom.Pose.Pose.Orientation.W;
    q1 = odom.Pose.Pose.Orientation.X;
    q2 = odom.Pose.Pose.Orientation.Y;
    q3 = odom.Pose.Pose.Orientation.Z;  
    odom_graph(i,3) = atan2(2*q1*q2+2*q0*q3,q1^2+q0^2-q3^2-q2^2);
    
    time_stamp(i,1) = imu.Header.Stamp.Sec + imu.Header.Stamp.Nsec*10^-9;
    time_stamp(i,2) = joint.Header.Stamp.Sec + imu.Header.Stamp.Nsec*10^-9;
    
    if i == 1
        joint_prev = joint.Position;
    end
    deltas = (joint.Position(1) - joint_prev(1) + joint.Position(2) - joint_prev(2))*r/2;
    deltaTheta = (joint.Position(1) - joint_prev(1) - joint.Position(2) + joint_prev(2))*r/b;
    joint_prev = joint.Position;
    
    Q_prev = diag([0.01, 0.01, 0.1]);
    
    P_priori = F_prev*P_post_prev*transpose(F_prev)+Q_prev;
    
    v = v_prev + imu.LinearAcceleration.X*dT;
    
    if abs(xhat_priori(3)-xhat_post_prev(3)) > pi
        if xhat_priori(3) > xhat_post_prev(3)
            xhat_priori(3) = xhat_priori(3) - 2*pi;
        else
            xhat_priori(3) = xhat_priori(3) + 2*pi;
        end
    end
%    H = [(xhat_priori(1)-xhat_post_prev(1))/(sqrt((xhat_priori(1)-xhat_post_prev(1))^2+(xhat_priori(2)-xhat_post_prev(2))^2)), (xhat_priori(2)-xhat_post_prev(2))/(sqrt((xhat_priori(1)-xhat_post_prev(1))^2+(xhat_priori(2)-xhat_post_prev(2))^2)), 0;
%        0, 0, 1;
%        0, 0, 1/dT;
%        (xhat_priori(1)-xhat_post_prev(1))/(dT*sqrt(((xhat_priori(1)-xhat_post_prev(1))/dT)^2+((xhat_priori(2)-xhat_post_prev(2))/dT)^2)), (xhat_priori(2)-xhat_post_prev(2))/dT*(sqrt(((xhat_priori(1)-xhat_post_prev(1))/dT)^2+((xhat_priori(2)-xhat_post_prev(2))/dT)^2)), 0];
    H = [(xhat_priori(1)-xhat_post_prev(1))/(sqrt((xhat_priori(1)-xhat_post_prev(1))^2+(xhat_priori(2)-xhat_post_prev(2))^2)), (xhat_priori(2)-xhat_post_prev(2))/(sqrt((xhat_priori(1)-xhat_post_prev(1))^2+(xhat_priori(2)-xhat_post_prev(2))^2)), 0;
        0, 0, 1;
        0, 0, 1/dT];

%    R = diag([0.2, 0.2, 0.1, 0.1]);
    R = diag([0.2, 0.2, 0.2]);
    
    K = P_priori*transpose(H)*inv(H*P_priori*transpose(H)+R);
    
%    y = [deltas; deltaTheta; imu.AngularVelocity.Z; v];
    y = [deltas; deltaTheta; imu.AngularVelocity.Z];
   
%    h = [sqrt((xhat_priori(1)-xhat_post_prev(1))^2+(xhat_priori(2)-xhat_post_prev(2))^2);
%        xhat_priori(3) - xhat_post_prev(3);
%        (xhat_priori(3) - xhat_post_prev(3))/dT;
%        sqrt(((xhat_priori(1)-xhat_post_prev(1))/dT)^2+((xhat_priori(2)-xhat_post_prev(2))/dT)^2)];
    h = [sqrt((xhat_priori(1)-xhat_post_prev(1))^2+(xhat_priori(2)-xhat_post_prev(2))^2);
        xhat_priori(3) - xhat_post_prev(3);
        (xhat_priori(3) - xhat_post_prev(3))/dT];
    
    if abs(y(3)-h(3)) > pi
        if y(3) > h(3)
            y(3) = y(3)-2*pi;
        else
            y(3) = y(3)+2*pi;
        end
    end
    
    xhat_post = xhat_priori + K*(y-h);
    xhat_post(3) = wrapToPi(xhat_post(3));
    xhat_post_prev = xhat_post;
    
    P_post_prev = (eye(3)-K*H)*P_priori;
    v_prev = v;
    
    xhat_graph(i,:) = xhat_post;
    time_stamp(i,3) = toc;
end
velmsg.Angular.Z = 0.0;
velmsg.Linear.X = 0.0;
send(control_pub,velmsg);
rosshutdown;

time_graph_prev = zeros(total_samples-1, 2);

for i = 2:total_samples
    time_graph_prev(i-1,1) = time_stamp(i,1) - time_stamp(i-1,1);
    time_graph_prev(i-1,2) = time_stamp(i,2) - time_stamp(i-1,2);
end

figure(1)
hold on
plot(1:total_samples-1, time_graph_prev(:,1))
plot(1:total_samples-1, time_graph_prev(:,2))
title('Sample time')
legend('IMU', 'Joint States')
grid on
hold off

figure(2)
hold on
plot(1:total_samples, xhat_graph(:,3))
plot(1:total_samples, odom_graph(:,3))
title('Ground Truth orientation versus EKF prediciton')
legend('Estimate', 'Ground Truth')
grid on
hold off

figure(3)
hold on
plot(xhat_graph(:,1), xhat_graph(:,2))
plot(odom_graph(:,1), odom_graph(:,2))
title('Ground Truth position versus EKF prediciton')
legend('Estimate', 'Ground Truth')
grid on
hold off

figure(4)
hold on
plot(1:total_samples, time_stamp(:,3))
title('EKF computation time')
grid on
hold off

figure(5)
hold on
plot(1:total_samples, imu_graph(:,1))
plot(1:total_samples, imu_graph(:,2))
title('IMU raw data')
legend('Acceleration.X', 'Velocity.Z')
grid on
hold off