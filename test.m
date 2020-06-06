clear variables, close all
clc

rosshutdown;
rosinit('http://192.168.1.102:11311');

control_pub = rospublisher('/cmd_vel');
velmsg = rosmessage(control_pub);
joint_sub = rossubscriber('/joint_states');
imu_sub = rossubscriber('/imu');
odom_sub =  rossubscriber('/odom');
pause(2)

total_samples = 1000;

xhat_graph = zeros(total_samples,11);
time_stamp = zeros(total_samples,3);
odom_graph = zeros(total_samples,3);

b = 0.287;  % wheelbase of wafflepi.
r = 0.033;
dT = 0.0669; % Update rate based on averaging of sample time. 0.038 = realbot

deltas = 0;
deltaTheta = 0;
xhat_priori = [0;0;0;0;0;0;0;0;0;0;0];
xhat_post_prev = [0.25; 0.25; 0; 3; sqrt(2*0.25^2); 0.1; 2.5; -0.3; 2.5; -2; 1];
F_prev = zeros(11);
Q_prev = zeros(11);
k = 0.0005;
P_priori = zeros(11);
P_post_prev = ([0.15; 0.15; 0.15; 0.15; 0.15; 0.15; 0.15; 0.15; 0.15; 0.15; 0.15] - [0.25; 0.25; 0; 3; sqrt(2*0.25^2); 0.1; 2.5; -0.3; 2.5; -2; 1])*transpose([0.15; 0.15; 0.15; 0.15; 0.15; 0.15; 0.15; 0.15; 0.15; 0.15; 0.15] - [0.25; 0.25; 0; 3; sqrt(2*0.25^2); 0.1; 2.5; -0.3; 2.5; -2; 1]);
v_prev = 0;
v = 0;
H = zeros(11,3);
R = zeros(3);
K = zeros(11,3);
y = [0;0;0;0;0;0;0;0;0;0;0];
xhat_post = [0;0;0;0;0;0;0;0;0;0;0];

velmsg.Angular.Z = 0.1;
velmsg.Linear.X = 0.2;
send(control_pub,velmsg);

for i = 1:total_samples
    
    if i == 500
        velmsg.Angular.Z = 0;
        velmsg.Linear.X = 0.05;
        send(control_pub,velmsg);
    end
    joint = receive(joint_sub);
    imu = receive(imu_sub);
    odom = receive(odom_sub);
    odom_graph(i,1) = odom.Pose.Pose.Position.X;
    odom_graph(i,2) = odom.Pose.Pose.Position.Y;
    q0 = odom.Pose.Pose.Orientation.W;
    q1 = odom.Pose.Pose.Orientation.X;
    q2 = odom.Pose.Pose.Orientation.Y;
    q3 = odom.Pose.Pose.Orientation.Z;  
    odom_graph(i,3) = atan2(2*q1*q2+2*q0*q3,q1^2+q0^2-q3^2-q2^2);
    
    time_stamp(i,1) = imu.Header.Stamp.Sec + imu.Header.Stamp.Nsec*10^-9;
    time_stamp(i,2) = joint.Header.Stamp.Sec + imu.Header.Stamp.Nsec*10^-9;
    
    tic
    if i == 1
        joint_prev = joint.Position;
    end
    deltas = (joint.Position(1) - joint_prev(1) + joint.Position(2) - joint_prev(2))*r/2;
    deltaTheta = (joint.Position(1) - joint_prev(1) - joint.Position(2) + joint_prev(2))*r/b;
    joint_prev = joint.Position;
    
    xhat_priori(1) = xhat_post_prev(1) + dT*velmsg.Linear.X*cos(xhat_post_prev(3));
    xhat_priori(2) = xhat_post_prev(2) + dT*velmsg.Linear.X*sin(xhat_post_prev(3));
    xhat_priori(3) = xhat_post_prev(3) + dT*velmsg.Angular.Z;
    xhat_priori(4) = atan2(0 - xhat_priori(2), 0 - xhat_priori(1)) - xhat_priori(3);
    xhat_priori(5) = sqrt((0 - xhat_priori(2))^2+(0 - xhat_priori(1))^2);
    xhat_priori(6) = atan2(0 - xhat_priori(2), 3 - xhat_priori(1)) - xhat_priori(3);
    xhat_priori(7) = sqrt((0 - xhat_priori(2))^2+(3 - xhat_priori(1))^2);
    xhat_priori(8) = atan2(3 - xhat_priori(2), 3 - xhat_priori(1)) - xhat_priori(3);
    xhat_priori(9) = sqrt((3 - xhat_priori(2))^2+(3 - xhat_priori(1))^2);
    xhat_priori(10) = atan2(3 - xhat_priori(2), 0 - xhat_priori(1)) - xhat_priori(3);
    xhat_priori(11) = sqrt((3 - xhat_priori(2))^2+(0 - xhat_priori(1))^2);
    
    F_prev = [1, 0, -dT*velmsg.Linear.X*sin(xhat_post_prev(3)), 0, 0, 0, 0, 0, 0, 0, 0;
        0, 1, dT*velmsg.Linear.X*cos(xhat_post_prev(3)), 0, 0, 0, 0, 0, 0, 0, 0;
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
        (xhat_priori(2)-0)/(((xhat_priori(2)-0)^2/(xhat_priori(1)-0)^2+1)*(xhat_priori(1)-0)^2), 1/(((xhat_priori(2)-0)^2/(xhat_priori(1)-0)^2+1)*(xhat_priori(1)-0)), (dT*velmsg.Linear.X*(dT*velmsg.Linear.X+cos(xhat_post_prev(3))*(xhat_post_prev(1)-0)+sin(xhat_post_prev(3))*(xhat_post_prev(2)-0)))/(((xhat_priori(2)-0)^2/(xhat_priori(1)-0)^2+1)*(xhat_priori(1)-0)^2)-1, 0, 0, 0, 0, 0, 0, 0, 0;
        (xhat_priori(1)-0)/sqrt((xhat_priori(1)-0)^2+(xhat_priori(2)-0)^2), (xhat_priori(2)-0)/sqrt((xhat_priori(1)-0)^2+(xhat_priori(2)-0)^2), (dT*(cos(xhat_post_prev(3))*(xhat_post_prev(2)-0)+sin(xhat_post_prev(3))*(xhat_post_prev(1)-0)))/sqrt((xhat_priori(1)-0)^2+(xhat_priori(2)-0)^2), 0, 0, 0, 0, 0, 0, 0, 0;
        (xhat_priori(2)-0)/(((xhat_priori(2)-0)^2/(xhat_priori(1)-3)^2+1)*(xhat_priori(1)-3)^2), 1/(((xhat_priori(2)-0)^2/(xhat_priori(1)-3)^2+1)*(xhat_priori(1)-3)), (dT*velmsg.Linear.X*(dT*velmsg.Linear.X+cos(xhat_post_prev(3))*(xhat_post_prev(1)-3)+sin(xhat_post_prev(3))*(xhat_post_prev(2)-0)))/(((xhat_priori(2)-0)^2/(xhat_priori(1)-3)^2+1)*(xhat_priori(1)-3)^2)-1, 0, 0, 0, 0, 0, 0, 0, 0;
        (xhat_priori(1)-3)/sqrt((xhat_priori(1)-3)^2+(xhat_priori(2)-0)^2), (xhat_priori(2)-0)/sqrt((xhat_priori(1)-3)^2+(xhat_priori(2)-0)^2), (dT*(cos(xhat_post_prev(3))*(xhat_post_prev(2)-0)+sin(xhat_post_prev(3))*(xhat_post_prev(1)-0)))/sqrt((xhat_priori(1)-3)^2+(xhat_priori(2)-0)^2), 0, 0, 0, 0, 0, 0, 0, 0;
        (xhat_priori(2)-3)/(((xhat_priori(2)-3)^2/(xhat_priori(1)-3)^2+1)*(xhat_priori(1)-3)^2), 1/(((xhat_priori(2)-3)^2/(xhat_priori(1)-3)^2+1)*(xhat_priori(1)-3)), (dT*velmsg.Linear.X*(dT*velmsg.Linear.X+cos(xhat_post_prev(3))*(xhat_post_prev(1)-3)+sin(xhat_post_prev(3))*(xhat_post_prev(2)-3)))/(((xhat_priori(2)-3)^2/(xhat_priori(1)-3)^2+1)*(xhat_priori(1)-3)^2)-1, 0, 0, 0, 0, 0, 0, 0, 0;
        (xhat_priori(1)-3)/sqrt((xhat_priori(1)-3)^2+(xhat_priori(2)-3)^2), (xhat_priori(2)-3)/sqrt((xhat_priori(1)-3)^2+(xhat_priori(2)-3)^2), (dT*(cos(xhat_post_prev(3))*(xhat_post_prev(2)-3)+sin(xhat_post_prev(3))*(xhat_post_prev(1)-3)))/sqrt((xhat_priori(1)-3)^2+(xhat_priori(2)-3)^2), 0, 0, 0, 0, 0, 0, 0, 0;
        (xhat_priori(2)-3)/(((xhat_priori(2)-3)^2/(xhat_priori(1)-0)^2+1)*(xhat_priori(1)-0)^2), 1/(((xhat_priori(2)-3)^2/(xhat_priori(1)-0)^2+1)*(xhat_priori(1)-0)), (dT*velmsg.Linear.X*(dT*velmsg.Linear.X+cos(xhat_post_prev(3))*(xhat_post_prev(1)-0)+sin(xhat_post_prev(3))*(xhat_post_prev(2)-3)))/(((xhat_priori(2)-3)^2/(xhat_priori(1)-0)^2+1)*(xhat_priori(1)-0)^2)-1, 0, 0, 0, 0, 0, 0, 0, 0;
        (xhat_priori(1)-0)/sqrt((xhat_priori(1)-0)^2+(xhat_priori(2)-3)^2), (xhat_priori(2)-3)/sqrt((xhat_priori(1)-0)^2+(xhat_priori(2)-3)^2), (dT*(cos(xhat_post_prev(3))*(xhat_post_prev(2)-3)+sin(xhat_post_prev(3))*(xhat_post_prev(1)-0)))/sqrt((xhat_priori(1)-0)^2+(xhat_priori(2)-3)^2), 0, 0, 0, 0, 0, 0, 0, 0];
    
    Q_prev = diag([k*abs(joint.Position(1)), k*abs(joint.Position(2)), 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]);
    
    P_priori = F_prev*P_post_prev*transpose(F_prev)+Q_prev;
    
    v = v_prev + imu.LinearAcceleration.X*dT;
    
    H = [(xhat_priori(1)-xhat_post_prev(1))/(sqrt((xhat_priori(1)-xhat_post_prev(1))^2+(xhat_priori(2)-xhat_post_prev(2))^2)), (xhat_priori(2)-xhat_post_prev(2))/(sqrt((xhat_priori(1)-xhat_post_prev(1))^2+(xhat_priori(2)-xhat_post_prev(2))^2)), 0, 0, 0, 0, 0, 0, 0, 0, 0;
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
        0, 0, 1/dT, 0, 0, 0, 0, 0, 0, 0, 0];
    
    R = diag([0.1, 0.2, 0.4]);
    
    K = P_priori*transpose(H)*inv(H*P_priori*transpose(H)+R);
    
    y = [deltas; deltaTheta; imu.AngularVelocity.Z];
    
    h = [sqrt((xhat_priori(1)-xhat_post_prev(1))^2+(xhat_priori(2)-xhat_post_prev(2))^2);
        xhat_priori(3) - xhat_post_prev(3);
        (xhat_priori(3) - xhat_post_prev(3))/dT];
    
    xhat_post = xhat_priori + K*(y-h);
    xhat_post_prev = xhat_post;
    
    P_post_prev = (eye(11)-K*H)*P_priori;
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
grid on
hold off

figure(2)
hold on
plot(1:total_samples, xhat_graph(:,3))
plot(1:total_samples, odom_graph(:,3))
grid on
hold off

figure(3)
hold on
plot(xhat_graph(:,1), xhat_graph(:,2))
plot(odom_graph(:,1), odom_graph(:,2))
grid on
hold off

figure(4)
hold on
plot(1:total_samples, time_stamp(:,3))
grid on
hold off

figure(5)
hold on
plot(xhat_graph(:,1), xhat_graph(:,2))
for i = 1:total_samples/50
    plot([xhat_graph(i*50,1) xhat_graph(i*50,5)*cos(xhat_graph(i*50,4))], [xhat_graph(i*50,2) xhat_graph(i*50,5)*sin(xhat_graph(i*50,4))], 'b')
    plot([xhat_graph(i*50,1) xhat_graph(i*50,7)*cos(xhat_graph(i*50,6))], [xhat_graph(i*50,2) xhat_graph(i*50,7)*sin(xhat_graph(i*50,6))], 'r')
    plot([xhat_graph(i*50,1) xhat_graph(i*50,9)*cos(xhat_graph(i*50,8))], [xhat_graph(i*50,2) xhat_graph(i*50,9)*sin(xhat_graph(i*50,8))], 'y')
    plot([xhat_graph(i*50,1) xhat_graph(i*50,11)*cos(xhat_graph(i*50,10))], [xhat_graph(i*50,2) xhat_graph(i*50,11)*sin(xhat_graph(i*50,10))], 'k')
end
grid on
hold off

figure(6)
hold on
plot(1:total_samples, xhat_graph(:,5))
plot(1:total_samples, xhat_graph(:,7))
plot(1:total_samples, xhat_graph(:,9))
plot(1:total_samples, xhat_graph(:,11))
grid on
hold off