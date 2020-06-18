clear variables, close all
clc

rosshutdown;
rosinit('http://192.168.1.102:11311');

control_pub = rospublisher('/cmd_vel');
velmsg = rosmessage(control_pub);
joint_sub = rossubscriber('/joint_states');
imu_sub = rossubscriber('/imu');
odom_sub =  rossubscriber('/odom');
scan_sub = rossubscriber('/scan');
%parpool;

total_samples = 200;

xhat_graph = zeros(total_samples,3);
time_stamp = zeros(total_samples,3);
odom_graph = zeros(total_samples,3);
imu_graph = zeros(total_samples,2);
y2_graph = zeros(total_samples,3);

b = 0.287;  % wheelbase of wafflepi.
r = 0.033;
dT = 0.2;
noLidar = 0;
ang_vel = 0;
ang_vel_prev = 0;

waypoints = [[0.25;0.25] [1.5;1.5] [1.5;2.5] [ 3;2.5] [1.5;0.25]];
Kp = 2;

waypointNav = 1;
xEnd = waypoints(1,waypointNav);
yEnd = waypoints(2,waypointNav);

deltas = 0;
deltaTheta = 0;
xhat_priori = [0;0;0];
xhat_post_prev = [0.25; 0.25; 0;]; %Initial position, [x, y, theta]
F_prev = zeros(3);
F_held = zeros(3,3,6);
Q_prev = diag([0.01, 0.01, 0.02]);
k = 0.0005;
P_priori = zeros(3);
P_post_prev = ([0.01; 0.01; 0.01] - xhat_post_prev)*transpose([0.01; 0.01; 0.01] - xhat_post_prev);
H = zeros(3);
H_held = zeros(3,3,6);
H2 = diag([1, 1, 1]);
R = diag([0.02, 0.02, 0.01]);
R2 = diag([0.03, 0.03, 0.02]);
K = zeros(3);
K_held = zeros(3,3,6);
K2 = zeros(3);
W = zeros(3);
y = [0;0;0];
y2 = [0;0;0];
xhat_post = [0;0;0];
xhat_s = [0;0;0];

lidar = receive(scan_sub);
[xhat_post_prev(1), xhat_post_prev(2), xhat_post_prev(3)] = lidarCalc(xhat_post_prev(1), xhat_post_prev(2), xhat_post_prev(3), lidar.Ranges, lidar.RangeMax, lidar.RangeMin, lidar.AngleIncrement);

velmsg.Angular.Z = 0;
velmsg.Linear.X = 0.15;
send(control_pub,velmsg);

rate = rateControl(5);
lidar = receive(scan_sub);
lidarFunc = parfeval(@lidarCalc, 3, xhat_post_prev(1), xhat_post_prev(2), xhat_post_prev(3), lidar.Ranges, lidar.RangeMax, lidar.RangeMin, lidar.AngleIncrement);

for i = 1:total_samples
    mod_loop = mod(i,5) + 1;
        
    % use below control logic to calculate phi_desired in four
    % quadrants
    if xEnd>xhat_post_prev(1)
        if yEnd>xhat_post_prev(2) % 1st quadrant
            phi_desired = atan((yEnd-xhat_post_prev(2))/(xEnd-xhat_post_prev(1)));
        else % 4th quadrant
            phi_desired = 2*pi - atan(abs(yEnd-xhat_post_prev(2))/abs(xEnd-xhat_post_prev(1)));
        end
    else
        if yEnd>xhat_post_prev(2) % 2nd quadrant
            phi_desired = pi - atan(abs(yEnd-xhat_post_prev(2))/abs(xEnd-xhat_post_prev(1)));
        else % 3rd quadrant
            phi_desired = pi + atan(abs(yEnd-xhat_post_prev(2))/abs(xEnd-xhat_post_prev(1)));
        end
    end        
        
    error = xhat_post_prev(3) - phi_desired;
    error = wrapToPi(error);
    ang_vel = -Kp * error;
    if abs(ang_vel_prev - ang_vel) > 0.03
        if ang_vel_prev - ang_vel > 0
            ang_vel = ang_vel_prev - 0.03;
        else
            ang_vel = ang_vel_prev + 0.03;
        end
    end
    if abs(ang_vel) > 0.3
        if ang_vel > 0
            ang_vel = 0.3;
        else
            ang_vel = -0.3;
        end
    end
    ang_vel_prev = ang_vel;

    velmsg.Angular.Z = ang_vel;
    send(control_pub,velmsg); 

    distance = sqrt((xEnd-xhat_post_prev(1))^2+(yEnd-xhat_post_prev(2))^2);

    % stop the robot if the distance between robot and goal is less than
    % 0.1m
    if abs(distance)<0.1
        if waypointNav < 5
            waypointNav = waypointNav + 1;
       end
       xEnd = waypoints(1,waypointNav)
       yEnd = waypoints(2,waypointNav)
    end
    
    xhat_priori(1) = xhat_post_prev(1) + dT*velmsg.Linear.X*cos(xhat_post_prev(3));
    xhat_priori(2) = xhat_post_prev(2) + dT*velmsg.Linear.X*sin(xhat_post_prev(3));
    xhat_priori(3) = xhat_post_prev(3) + dT*velmsg.Angular.Z;
    
    if i == 1
        xhat_s = xhat_priori;
    end
    
    F_prev = [1, 0, -dT*velmsg.Linear.X*sin(xhat_post_prev(3));
        0, 1, dT*velmsg.Linear.X*cos(xhat_post_prev(3));
        0, 0, 1];
    
    F_held(:,:,mod_loop) = F_prev;
    
    joint = receive(joint_sub);
    imu = receive(imu_sub);
    odom = receive(odom_sub);
    
    imu_graph(i,1) = imu.LinearAcceleration.X;
    imu_graph(i,2) = imu.AngularVelocity.Z;

    odom_graph(i,1) = odom.Pose.Pose.Position.X;
    odom_graph(i,2) = odom.Pose.Pose.Position.Y;
    q0 = odom.Pose.Pose.Orientation.W;
    q1 = odom.Pose.Pose.Orientation.X;
    q2 = odom.Pose.Pose.Orientation.Y;
    q3 = odom.Pose.Pose.Orientation.Z;  
    odom_graph(i,3) = atan2(2*q1*q2+2*q0*q3,q1^2+q0^2-q3^2-q2^2);
    
    time_stamp(i,1) = imu.Header.Stamp.Sec + imu.Header.Stamp.Nsec*10^-9;
    time_stamp(i,2) = joint.Header.Stamp.Sec + imu.Header.Stamp.Nsec*10^-9;
    
    if i > 1
        dT = time_stamp(i,1) - time_stamp(i-1,1);
    end
    if i == 1
        joint_prev = joint.Position;
    end
    
    deltas = (joint.Position(1) - joint_prev(1) + joint.Position(2) - joint_prev(2))*r/2;
    deltaTheta = (joint.Position(1) - joint_prev(1) - joint.Position(2) + joint_prev(2))*r/b;
    joint_prev = joint.Position;
    
    Q_prev = diag([velmsg.Linear.X + 0.02, velmsg.Linear.X + 0.02, velmsg.Angular.Z + 0.2]);
    
    P_priori = F_prev*P_post_prev*transpose(F_prev)+Q_prev;
    
    if abs(xhat_priori(3)-xhat_post_prev(3)) > pi
        if xhat_priori(3) > xhat_post_prev(3)
            xhat_priori(3) = xhat_priori(3) - 2*pi;
        else
            xhat_priori(3) = xhat_priori(3) + 2*pi;
        end
    end
    
    H = [(xhat_priori(1)-xhat_post_prev(1))/(sqrt((xhat_priori(1)-xhat_post_prev(1))^2+(xhat_priori(2)-xhat_post_prev(2))^2)), (xhat_priori(2)-xhat_post_prev(2))/(sqrt((xhat_priori(1)-xhat_post_prev(1))^2+(xhat_priori(2)-xhat_post_prev(2))^2)), 0;
        0, 0, 1;
        0, 0, 1/dT];
    H_held(:,:,mod_loop) = H;
    
    K = P_priori*transpose(H)*inv(H*P_priori*transpose(H)+R);
    K_held(:,:,mod_loop) = K;
    
    y = [deltas; deltaTheta; imu.AngularVelocity.Z];
   
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
    
    if mod(i, 5) == 0 && noLidar == 0
        [y2(1), y2(2), y2(3)] = fetchOutputs(lidarFunc);
        y2_graph(i,:) = y2;
        
        lidar = receive(scan_sub);
        lidarFunc = parfeval(@lidarCalc, 3, xhat_post_prev(1), xhat_post_prev(2), xhat_post_prev(3), lidar.Ranges, lidar.RangeMax, lidar.RangeMin, lidar.AngleIncrement);

        if isnan(y2)
            xhat_post = xhat_priori + K*(y-h);
            P_post_prev = (eye(3) - K*H)*P_priori;
        else
            K2 = P_priori*transpose(H2)*inv(H2*P_priori*transpose(H2)+R2);
            P_post_prev = (eye(3) - K2*H2)*P_priori;
            W = (eye(3) - K_held(:,:,2)*H_held(:,:,2))*F_held(:,:,1);
            for j = 2:5
                W = W * (eye(3) - K_held(:,:,j+1)*H_held(:,:,j+1))*F_held(:,:,j);
            end

            xhat_post = xhat_priori + K*(y-h) + W*K2*(y2-H2*xhat_s);
        end
        xhat_s = xhat_priori;
    else
        xhat_post = xhat_priori + K*(y-h);
        P_post_prev = (eye(3) - K*H)*P_priori;
    end
    
    xhat_post(3) = wrapToPi(xhat_post(3));
    xhat_post_prev = xhat_post;
    
    xhat_graph(i,:) = xhat_post;
    
    waitfor(rate);
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
plot([1:total_samples] - 5, y2_graph(:,3), '*')
title('Ground Truth orientation versus EKF prediciton')
legend('Estimate', 'Ground Truth', 'Beacon')
grid on
hold off

figure(3)
hold on
plot(xhat_graph(:,1), xhat_graph(:,2))
plot(odom_graph(:,1), odom_graph(:,2))
plot(y2_graph(:,1), y2_graph(:,2), '*')
title('Ground Truth position versus EKF prediciton')
legend('Estimate', 'Ground Truth', 'Beacon')
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