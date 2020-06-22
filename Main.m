% ---------------------------
% Author: Michael Duke
% Description: Localisation script for TurtleBot3 Waffle Pi using a
%  cascading Extended Kalman Filter. A primary dead
%  reckoning estimate, and a slower secondary absolute estimate using
%  a beacon based LiDAR.
% ---------------------------

clear variables, close all
clc

%10.128.0.216 = Unisa-mech,
rosshutdown;
rosinit('http://192.168.1.102:11311');

% ROS Node Inits
S = 'Subscribing to topics';
disp(S)
control_pub = rospublisher('/cmd_vel');
velmsg = rosmessage(control_pub);
joint_sub = rossubscriber('/joint_states');
imu_sub = rossubscriber('/imu');
odom_sub =  rossubscriber('/odom');
scan_sub = rossubscriber('/scan');
S = 'Starting parallel pool';
disp(S)
poolobj = gcp('nocreate');
if isempty(poolobj)
    poolobj = parpool(2);
    poolsize = poolobj.NumWorkers;
else
    poolsize = poolobj.NumWorkers;
end
S = 'Parallel pool started with two workers';

% Time of running, assuming 5Hz, total running time is total_samples/5
% seconds.
total_samples = 400;

% Datalogging graphs
xhat_graph = zeros(total_samples,3);
time_stamp = zeros(total_samples,3);
odom_graph = zeros(total_samples,3);
imu_graph = zeros(total_samples,2);
y2_graph = zeros(total_samples,3);

model_graph = zeros(total_samples,3);
joint_graph = zeros(total_samples,3);
imuphi_graph = zeros(total_samples,1);
lidar_graph = zeros(total_samples,3);

% Init variables 
b = 0.287;          % Wheelbase of waffle pi.
r = 0.033;          % Radius of waffle pi wheels.
dT = 0.2;           % Time since last sample.
ang_vel = 0;        % Angular velocity.
ang_vel_prev = 0;   % Previous samples angular velocity.
ang_vel_max = 0.5;  % |max| value of ang_vel.
ang_vel_slew = 0.06;% |max| slew rate of ang_vel.
lin_vel = 0;        % Linear velocity.
lin_vel_max = 0.15; % |max| lin_vel.
% List of waypoints to drive to sequential.
waypoints = [[0.25;0.25] [1.5;1.5] [1.5;2.5] [ 3;2.5] [1.5;0.25]];
Kp = 1.5;           % Proportional gain for control loop.
noLidar = 0;        % Flag to run code Dead Reckoning only.
calRun = 1;         % Flag to run calibration only run.
waypointNav = 1;    % Loop control for what waypoint to start at.
xEnd = waypoints(1,waypointNav);
yEnd = waypoints(2,waypointNav);

% EKF Init variables
deltas = 0;             % Average distance wheels travelled.
deltaTheta = 0;         % Change of estimation based off deltas.
xhat_priori = [0;0;0];  % x priori estimate [x, y, theta].
xhat_post_prev = [0.25; 0.25; 0;]; %previous x posterior [x, y, theta].
F_prev = zeros(3);      % F matrix, linear.
F_held = zeros(3,3,6);  % F matrix matrix, for recalc of secondary.
Q_prev = diag([5.5, 2.7, 71]*10^-6); % Q covariance matrix, process noise.
P_priori = zeros(3);    % Priori error covariance matrix
P_post_prev = ([0.01; 0.01; 0.01] - xhat_post_prev)...
    *transpose([0.01; 0.01; 0.01] - xhat_post_prev); % Initial post error
H = zeros(3);           % H matrix, linear, match measure to states.
H_held = zeros(3,3,6);  % H matrix matrix, for recalc of secondary.
H2 = diag([1, 1, 1]);   % H matrix of secondary measurements.
R = diag([1.6, 7.2, 5.7]*10^-5); % R covariance matrix, measurement noise.
R2 = diag([0.005, 0.005, 0.005]);  % R covariance matrix, for secondary.
K = zeros(3);           % Kalman gain
K_held = zeros(3,3,6);  % Kalman gain matrix, for recalc of secondary.
K2 = zeros(3);          % Kalman gain from secondary.
W = zeros(3);           % Delayed measurement adjust matrix.
y = [0;0;0];        % Measurement inputs [deltas, deltaTheta, IMUTheta].
y2 = [0;0;0];       % Measurement inputs [LiDAR_x, LiDAR_y, LiDAR_Theta].
xhat_post = [0;0;0];    % Posterior state estimate.
xhat_s = [0;0;0];       % State estimate at secondary measurement time.

% Initial lidar scan to estimate initial pose.
lidar = receive(scan_sub);
[xhat_post_prev(1), xhat_post_prev(2), xhat_post_prev(3)] =...
    lidarCalc(xhat_post_prev(1), xhat_post_prev(2), xhat_post_prev(3),...
    lidar.Ranges, lidar.RangeMax, lidar.RangeMin, lidar.AngleIncrement);

% Stop robot (just in case)
velmsg.Angular.Z = 0;
velmsg.Linear.X = 0;
send(control_pub,velmsg);

% Sets rate of primary measurements in Hz.
rate = rosrate(5);

% First secondary measurement.
lidar = receive(scan_sub);
lidarFunc = parfeval(@lidarCalc, 3, xhat_post_prev(1),...
    xhat_post_prev(2), xhat_post_prev(3), lidar.Ranges,...
    lidar.RangeMax, lidar.RangeMin, lidar.AngleIncrement);

% ---------------------------
% MAIN LOOP
% ---------------------------

for i = 1:total_samples
    % Control for held primary measurements.
    mod_loop = mod(i,5) + 1;
        
    % Calculate phi_desired in four quadrants and wrap to pi.
    if xEnd>xhat_post_prev(1)
        if yEnd>xhat_post_prev(2) % 1st quadrant
            phi_desired = atan((yEnd-xhat_post_prev(2))/...
                (xEnd-xhat_post_prev(1)));
        else % 4th quadrant
            phi_desired = 2*pi - atan(abs(yEnd-xhat_post_prev(2))/...
                abs(xEnd-xhat_post_prev(1)));
        end
    else
        if yEnd>xhat_post_prev(2) % 2nd quadrant
            phi_desired = pi - atan(abs(yEnd-xhat_post_prev(2))/...
                abs(xEnd-xhat_post_prev(1)));
        else % 3rd quadrant
            phi_desired = pi + atan(abs(yEnd-xhat_post_prev(2))/...
                abs(xEnd-xhat_post_prev(1)));
        end
    end        
    
    % Calculate error and adjust ang_vel and lin_vel with slew control.
    error = xhat_post_prev(3) - phi_desired;
    error = wrapToPi(error);
    ang_vel = -Kp * error;
    if abs(ang_vel_prev - ang_vel) > ang_vel_slew
        if ang_vel_prev - ang_vel > 0
            ang_vel = ang_vel_prev - ang_vel_slew;
        else
            ang_vel = ang_vel_prev + ang_vel_slew;
        end
    end
    if abs(ang_vel) > ang_vel_max
        if ang_vel > 0
            ang_vel = ang_vel_max;
        else
            ang_vel = -ang_vel_max;
        end
    end
    ang_vel_prev = ang_vel;
    
    if ang_vel == 0
        lin_vel = lin_vel_max;
    else
        lin_vel = 1/(20*abs(ang_vel));
        if lin_vel > lin_vel_max
            lin_vel = lin_vel_max;
        end
    end

    velmsg.Angular.Z = ang_vel;
    velmsg.Linear.X = lin_vel;
    send(control_pub,velmsg); 

    % next waypoint if the distance between robot and goal is less than
    % 0.1m.
    distance = sqrt((xEnd - xhat_post_prev(1))^2 + (yEnd - xhat_post_prev(2))^2);
    if abs(distance)<0.1
        if waypointNav < 5
            waypointNav = waypointNav + 1;
       end
       xEnd = waypoints(1,waypointNav)
       yEnd = waypoints(2,waypointNav)
    end
    
    if calRun == 0
        % priori state estimate.
        xhat_priori(1) = xhat_post_prev(1) +...
            dT*velmsg.Linear.X*cos(xhat_post_prev(3));
        xhat_priori(2) = xhat_post_prev(2) +...
            dT*velmsg.Linear.X*sin(xhat_post_prev(3));
        xhat_priori(3) = xhat_post_prev(3) +...
            dT*velmsg.Angular.Z;
    
        % use current estimate as previous on first loop.
        if i == 1
            xhat_s = xhat_priori;
        end

        % priori state estimate linear.
        F_prev = [1, 0, -dT*velmsg.Linear.X*sin(xhat_post_prev(3));
            0, 1, dT*velmsg.Linear.X*cos(xhat_post_prev(3));
            0, 0, 1];
        F_held(:,:,mod_loop) = F_prev;
    end
    
    % receive from topics.
    joint = receive(joint_sub);
    imu = receive(imu_sub);
    odom = receive(odom_sub);
    if calRun == 1
        lidar = receive(scan_sub);
    end
    
    % calcs for graphing only.
    imu_graph(i,1) = imu.LinearAcceleration.X;
    imu_graph(i,2) = imu.AngularVelocity.Z;
    odom_graph(i,1) = odom.Pose.Pose.Position.X;
    odom_graph(i,2) = odom.Pose.Pose.Position.Y;
    q0 = odom.Pose.Pose.Orientation.W;
    q1 = odom.Pose.Pose.Orientation.X;
    q2 = odom.Pose.Pose.Orientation.Y;
    q3 = odom.Pose.Pose.Orientation.Z;  
    odom_graph(i,3) = atan2(2*q1*q2+2*q0*q3,q1^2+q0^2-q3^2-q2^2);
    time_stamp(i,1) = imu.Header.Stamp.Sec +...
        imu.Header.Stamp.Nsec*10^-9;
    time_stamp(i,2) = joint.Header.Stamp.Sec +...
        imu.Header.Stamp.Nsec*10^-9;
    if i > 1
        dT = time_stamp(i,1) - time_stamp(i-1,1);
    end
    if i == 1
        joint_prev = joint.Position;
    end
    
    % joint states pre-calcs.
    deltas = (joint.Position(1) - joint_prev(1) +...
        joint.Position(2) - joint_prev(2))*r/2;
    deltaTheta = (joint.Position(1) - joint_prev(1) -...
        joint.Position(2) + joint_prev(2))*r/b;
    joint_prev = joint.Position;
    
    if calRun == 1
        % Actual Position
        xhat_post(1) = odom.Pose.Pose.Position.X;
        xhat_post(2) = odom.Pose.Pose.Position.Y;
        xhat_post(3) = odom_graph(i,3);
        
        % Model Estimate
        model_graph(i,1) = xhat_post_prev(1) +...
            dT*velmsg.Linear.X*cos(xhat_post_prev(3));
        model_graph(i,2) = xhat_post_prev(2) +...
            dT*velmsg.Linear.X*sin(xhat_post_prev(3));
        model_graph(i,3) = xhat_post_prev(3) +...
            dT*velmsg.Angular.Z;
        
        % Joint State Estimate
        joint_graph(i,1) = xhat_post_prev(1) +...
            deltas*cos(xhat_post_prev(3) + deltaTheta/2);
        joint_graph(i,2) = xhat_post_prev(2) +...
            deltas*sin(xhat_post_prev(3) + deltaTheta/2);
        joint_graph(i,3) = xhat_post_prev(3) + deltaTheta;
        
        % IMU Estimate
        imuphi_graph(i,1) = xhat_post_prev(3) +...
            imu.AngularVelocity.Z*dT;
        
        % LiDAR Estimate
        [lidar_graph(i,1), lidar_graph(i,2), lidar_graph(i,3)] =...
            lidarCalc(xhat_post_prev(1),...
            xhat_post_prev(2), xhat_post_prev(3), lidar.Ranges,...
            lidar.RangeMax, lidar.RangeMin, lidar.AngleIncrement);
    else
        % Calculate priori error.
        P_priori = F_prev*P_post_prev*transpose(F_prev)+Q_prev;

        if abs(xhat_priori(3)-xhat_post_prev(3)) > pi
            if xhat_priori(3) > xhat_post_prev(3)
                xhat_priori(3) = xhat_priori(3) - 2*pi;
            else
                xhat_priori(3) = xhat_priori(3) + 2*pi;
            end
        end

        % Measurement matrix, linear.
        H = [(xhat_priori(1)-xhat_post_prev(1))/...
                (sqrt((xhat_priori(1)-xhat_post_prev(1))^2+...
                (xhat_priori(2)-xhat_post_prev(2))^2)),...
                (xhat_priori(2)-xhat_post_prev(2))/...
                (sqrt((xhat_priori(1)-xhat_post_prev(1))^2+...
                (xhat_priori(2)-xhat_post_prev(2))^2)), 0;
            0, 0, 1;
            0, 0, 1/dT];
        H_held(:,:,mod_loop) = H;

        % Kalman gain calc.
        K = P_priori*transpose(H)*inv(H*P_priori*transpose(H)+R);
        K_held(:,:,mod_loop) = K;

        % Primary measurements vector.
        y = [deltas; deltaTheta; imu.AngularVelocity.Z]; 
        h = [sqrt((xhat_priori(1)-xhat_post_prev(1))^2+...
            (xhat_priori(2)-xhat_post_prev(2))^2);
            xhat_priori(3) - xhat_post_prev(3);
            (xhat_priori(3) - xhat_post_prev(3))/dT];
        if abs(y(3)-h(3)) > pi
            if y(3) > h(3)
                y(3) = y(3)-2*pi;
            else
                y(3) = y(3)+2*pi;
            end
        end

        % Secondary measurements vector (every 5 samples)
        % and posterioir state estimate.
        if mod(i, 5) == 0 && noLidar == 0
            [y2(1), y2(2), y2(3)] = fetchOutputs(lidarFunc);
            y2_graph(i,:) = y2;

            lidar = receive(scan_sub);
            lidarFunc = parfeval(@lidarCalc, 3, xhat_post_prev(1),...
                xhat_post_prev(2), xhat_post_prev(3), lidar.Ranges,...
                lidar.RangeMax, lidar.RangeMin, lidar.AngleIncrement);

            if isnan(y2)
                xhat_post = xhat_priori + K*(y-h);
                P_post_prev = (eye(3) - K*H)*P_priori;
            else
                K2 = P_priori*transpose(H2)*...
                    inv(H2*P_priori*transpose(H2)+R2);
                P_post_prev = (eye(3) - K2*H2)*P_priori;
                W = (eye(3) - K_held(:,:,2)*H_held(:,:,2))*F_held(:,:,1);
                for j = 2:5
                    W = W * (eye(3) - K_held(:,:,j+1)*...
                        H_held(:,:,j+1))*F_held(:,:,j);
                end

                xhat_post = xhat_priori + K*(y-h) + W*K2*(y2-H2*xhat_s);
            end
            xhat_s = xhat_priori;
        else
            xhat_post = xhat_priori + K*(y-h);
            P_post_prev = (eye(3) - K*H)*P_priori;
        end
    end
    xhat_post(3) = wrapToPi(xhat_post(3));
    xhat_post_prev = xhat_post;
    
    % State estimate for graphing.
    xhat_graph(i,:) = xhat_post;
    
    waitfor(rate);
end
% ------------------------
% END OF MAIN LOOP
% ------------------------

% Stop bot.
velmsg.Angular.Z = 0.0;
velmsg.Linear.X = 0.0;
send(control_pub,velmsg);
rosshutdown;

% ------------------------
% POST GRAPHING
% ------------------------
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

if calRun == 1
    model_error = odom_graph - model_graph;
    model_error(:,3) = wrapToPi(model_error(:,3));
    joint_error = odom_graph - joint_graph;
    joint_error(:,3) = wrapToPi(joint_error(:,3));
    imu_error = odom_graph(:,3) - imuphi_graph;
    imu_error = wrapToPi(imu_error);
    lidar_error = odom_graph - lidar_graph;
    lidar_error(:,3) = wrapToPi(lidar_error(:,3));
    
    cov_model_err_x = cov(model_error(:,1));
    cov_model_err_y = cov(model_error(:,2));
    cov_model_err_theta = cov(model_error(:,3));
    
    cov_joint_err_x = cov(joint_error(:,1));
    cov_joint_err_y = cov(joint_error(:,2));
    cov_joint_err_theta = cov(model_error(:,3));

    cov_imu_err_theta = cov(imu_error);
    
    cov_lidar_err_x = cov(lidar_error(:,1));
    cov_lidar_err_y = cov(lidar_error(:,2));
    cov_lidar_err_theta = cov(lidar_error(:,3));
    
    figure(2)
    set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
    subplot(4,3,1)
    hold on
    plot(1:total_samples, model_error(:,1))
    title('Error of model, x estimate')
    xlabel('Sample')
    ylabel('x Error (m)')
    grid on
    hold off
    
    subplot(4,3,2)
    hold on
    plot(1:total_samples, model_error(:,2))
    title('Error of model, y estimate')
    xlabel('Sample')
    ylabel('x Error (m)')
    grid on
    hold off
    
    subplot(4,3,3)
    hold on
    plot(1:total_samples, model_error(:,3))
    title('Error of model, theta estimate')
    xlabel('Sample')
    ylabel('\theta Error (rads)')
    grid on
    hold off
    
    subplot(4,3,4)
    hold on
    plot(1:total_samples, joint_error(:,1))
    title('Error of joint states, x estimate')
    xlabel('Sample')
    ylabel('x Error (m)')
    grid on
    hold off
    
    subplot(4,3,5)
    hold on
    plot(1:total_samples, joint_error(:,2))
    title('Error of joint states, y estimate')
    xlabel('Sample')
    ylabel('x Error (m)')
    grid on
    hold off
    
    subplot(4,3,6)
    hold on
    plot(1:total_samples, joint_error(:,3))
    title('Error of joint states, theta estimate')
    xlabel('Sample')
    ylabel('\theta Error (rads)')
    grid on
    hold off
    
    subplot(4,3,9)
    hold on
    plot(1:total_samples, imu_error(:,1))
    title('Error of IMU, theta estimate')
    xlabel('Sample')
    ylabel('\theta Error (rads)')
    grid on
    hold off
    
    subplot(4,3,10)
    hold on
    plot(1:total_samples, lidar_error(:,1))
    title('Error of lidar, x estimate')
    xlabel('Sample')
    ylabel('x Error (m)')
    grid on
    hold off
    
    subplot(4,3,11)
    hold on
    plot(1:total_samples, lidar_error(:,2))
    title('Error of lidar, y estimate')
    xlabel('Sample')
    ylabel('x Error (m)')
    grid on
    hold off
    
    subplot(4,3,12)
    hold on
    plot(1:total_samples, lidar_error(:,3))
    title('Error of lidar, theta estimate')
    xlabel('Sample')
    ylabel('\theta Error (rads)')
    grid on
    hold off
    print('errorAbs', '-dpng', '-r600')
else
    figure(2)
    hold on
    plot(1:total_samples, xhat_graph(:,3))
    plot(1:total_samples, odom_graph(:,3))
    plot([5:5:total_samples] - 5, y2_graph(5:5:end,3), '*')
    %title('Ground Truth orientation versus EKF prediciton')
    legend('Estimate', 'Ground Truth', 'Beacon')
    xlabel('Sample')
    ylabel('Yaw (rads)')
    grid on
    hold off
    print('yawSimu', '-dpng', '-r600')

    figure(3)
    hold on
    plot(xhat_graph(:,1), xhat_graph(:,2))
    plot(odom_graph(:,1), odom_graph(:,2))
    plot(y2_graph(5:5:end,1), y2_graph(5:5:end,2), '*')
    %title('Ground Truth position versus EKF prediciton')
    legend('Estimate', 'Ground Truth', 'Beacon')
    xlabel('x-position (m)')
    ylabel('y-position (m)')
    grid on
    hold off
    print('posSimu', '-dpng', '-r600')
    
    thetaError = xhat_graph(:,3)-odom_graph(:,3);
    thetaError(340) = thetaError(340) + 2*pi;
    figure(4)
    subplot(3,1,1)
    hold on
    plot(1:total_samples, xhat_graph(:,1)-odom_graph(:,1))
    xlabel('Sample')
    ylabel('x Error (m)')
    grid on
    hold off
    
    subplot(3,1,2)
    hold on
    plot(1:total_samples, xhat_graph(:,2)-odom_graph(:,2))
    xlabel('Sample')
    ylabel('y Error (m)')
    grid on
    hold off
    
    subplot(3,1,3)
    hold on
    plot(1:total_samples, thetaError)
    xlabel('Sample')
    ylabel('\theta Error (rads)')
    grid on
    hold off
    print('Error', '-dpng', '-r600')
end