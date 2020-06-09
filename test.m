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
pause(2)

total_samples = 100;

xhat_graph = zeros(total_samples,3);
time_stamp = zeros(total_samples,3);
odom_graph = zeros(total_samples,3);

b = 0.287;  % wheelbase of wafflepi.
r = 0.033;
time_prev = 0;
dT = 0.25; % Update rate based on averaging of sample time. 0.038 = realbot

deltas = 0;
deltaTheta = 0;
xhat_priori = [0;0;0];
xhat_post_prev = [0.25; 0.25; 0;]; %Initial position, [x, y, theta]
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

beacons = [0, 0; 3, 0; 3, 3; 0, 3]; %4x2 (x,y)
beaconestimate = zeros(floor(total_samples/100), 8); % used for graphing

lidar = receive(scan_sub);
phiTolerance = 10;
lidar.Ranges(lidar.Ranges == Inf) = NaN;
lidar.Ranges(lidar.Ranges > lidar.RangeMax) = NaN;
lidar.Ranges(lidar.Ranges < lidar.RangeMin) = NaN;

lidar1Rough = atan2((beacons(1,2) - xhat_post_prev(2)),(beacons(1,1) - xhat_post_prev(1))) - xhat_post_prev(3);
if lidar1Rough < 0
    lidar1Rough =+ 2*pi;
end
lidar1RoughLow = round(lidar1Rough/0.0175) - phiTolerance;
if lidar1RoughLow < 1
    lidar1RoughLow = 1;
end
lidar1RoughHigh = round(lidar1Rough/0.0175) + phiTolerance;
if lidar1RoughHigh > 360
    lidar1RoughHigh = 360;
end

lidar1Ranges = lidar.Ranges(lidar1RoughLow:1:lidar1RoughHigh);
[lidar1Min, lidar1MinInd] = min(lidar1Ranges);
beaconMeasureOriginal = [lidar1MinInd*lidar.AngleIncrement, lidar1Min+0.125;
    0, 0;
    0, 0;
    0, 0];
if any(lidar1Min)
    xhat_post_prev(3) = lidar1Rough - beaconMeasureOriginal(1,1);
end

lidar2Rough = atan2((beacons(2,2) - xhat_post_prev(2)),(beacons(2,1) - xhat_post_prev(1))) - xhat_post_prev(3);
if lidar2Rough < 0
    lidar2Rough =+ 2*pi;
end
lidar2RoughLow = round(lidar2Rough/0.0175) - phiTolerance;
if lidar2RoughLow < 1
    lidar2RoughLow = 1;
end
lidar2RoughHigh = round(lidar2Rough/0.0175) + phiTolerance;
if lidar2RoughHigh > 360
    lidar2RoughHigh = 360;
end
lidar2Ranges = lidar.Ranges(lidar2RoughLow:1:lidar2RoughHigh);
[lidar2Min, lidar2MinInd] = min(lidar2Ranges);

lidar3Rough = atan2((beacons(3,2) - xhat_post_prev(2)),(beacons(3,1) - xhat_post_prev(1))) - xhat_post_prev(3);
if lidar3Rough < 0
    lidar3Rough =+ 2*pi;
end
lidar3RoughLow = round(lidar3Rough/0.0175) - phiTolerance;
if lidar3RoughLow < 1
    lidar3RoughLow = 1;
end
lidar3RoughHigh = round(lidar3Rough/0.0175) + phiTolerance;
if lidar3RoughHigh > 360
    lidar3RoughHigh = 360;
end
lidar3Ranges = lidar.Ranges(lidar3RoughLow:1:lidar3RoughHigh);
[lidar3Min, lidar3MinInd] = min(lidar3Ranges);

lidar4Rough = atan2((beacons(4,2) - xhat_post_prev(2)),(beacons(4,1) - xhat_post_prev(1))) - xhat_post_prev(3);
if lidar4Rough < 0
    lidar4Rough =+ 2*pi;
end
lidar4RoughLow = round(lidar4Rough/0.0175) - phiTolerance;
if lidar4RoughLow < 1
    lidar4RoughLow = 1;
end
lidar4RoughHigh = round(lidar4Rough/0.0175) + phiTolerance;
if lidar4RoughHigh > 360
    lidar4RoughHigh = 360;
end
lidar4Ranges = lidar.Ranges(lidar4RoughLow:1:lidar4RoughHigh);
[lidar4Min, lidar4MinInd] = min(lidar4Ranges);
% phi, radius. [4x2]
beaconMeasureOriginal = [lidar1MinInd*lidar.AngleIncrement, lidar1Min+0.125;
    (lidar2MinInd+lidar2Rough-phiTolerance)*lidar.AngleIncrement, lidar2Min+0.125;
    (lidar3MinInd+lidar4Rough-phiTolerance)*lidar.AngleIncrement, lidar3Min+0.125;
    (lidar4MinInd+lidar4Rough-phiTolerance)*lidar.AngleIncrement, lidar4Min+0.125];
beaconMeasure = beaconMeasureOriginal;
closestBeacon = zeros(4,1);
beacNums = 0;
while any(beaconMeasure(:,2))
    beacNums = beacNums + 1;
    [~, closestBeacon(beacNums)] = min(beaconMeasure(:,2));
    beaconMeasure(closestBeacon(beacNums),2) = NaN;
end

lidEstimateCoords = zeros(6,2); % (x,y)

if beacNums > 2
    for i = 1:2
        circDist = sqrt((beacons(closestBeacon(i),1)-beacons(closestBeacon(i+1),1))^2+(beacons(closestBeacon(i),2)-beacons(closestBeacon(i+1),2))^2);
        aquad = (beaconMeasureOriginal(closestBeacon(i),2)^2-beaconMeasureOriginal(closestBeacon(i+1),2)^2+circDist^2)/(2*circDist);
        if beaconMeasureOriginal(closestBeacon(i),2)+beaconMeasureOriginal(closestBeacon(i+1),2) >= circDist            
            hquad = sqrt(beaconMeasureOriginal(closestBeacon(i),2)^2 - aquad^2);
            xMiddle = beacons(closestBeacon(i),1) + aquad*(beacons(closestBeacon(i+1),1)-beacons(closestBeacon(i),1))/circDist;
            yMiddle = beacons(closestBeacon(i),2) + aquad*(beacons(closestBeacon(i+1),2)-beacons(closestBeacon(i),2))/circDist;
            
            lidEstimateCoords(i*2-1, 2) = xMiddle + hquad*(beacons(closestBeacon(i+1),2)-beacons(closestBeacon(i),2))/circDist;
            lidEstimateCoords(i*2-1, 1) = yMiddle - hquad*(beacons(closestBeacon(i+1),1)-beacons(closestBeacon(i),1))/circDist;
            lidEstimateCoords(i*2, 2) = xMiddle - hquad*(beacons(closestBeacon(i+1),2)-beacons(closestBeacon(i),2))/circDist;
            lidEstimateCoords(i*2, 1) = yMiddle + hquad*(beacons(closestBeacon(i+1),1)-beacons(closestBeacon(i),1))/circDist;
        else
            lidEstimateCoords(i*2-1, 2) = beacons(closestBeacon(i),1) + aquad*(beacons(closestBeacon(i+1),1)-beacons(closestBeacon(i),1))/circDist;
            lidEstimateCoords(i*2, 2) = lidEstimateCoords(i*2-1, 2);
            lidEstimateCoords(i*2-1, 1) = beacons(closestBeacon(i),2) + aquad*(beacons(closestBeacon(i+1),2)-beacons(closestBeacon(i),2))/circDist;
            lidEstimateCoords(i*2, 1) = lidEstimateCoords(i*2, 1);
        end
        circDist = sqrt((beacons(closestBeacon(1),1)-beacons(closestBeacon(3),1))^2+(beacons(closestBeacon(1),2)-beacons(closestBeacon(3),2))^2);
        aquad = (beaconMeasureOriginal(closestBeacon(1),2)^2-beaconMeasureOriginal(closestBeacon(3),2)^2+circDist^2)/(2*circDist);
        if beaconMeasureOriginal(closestBeacon(1),2)+beaconMeasureOriginal(closestBeacon(3),2) >= circDist
            hquad = sqrt(beaconMeasureOriginal(closestBeacon(1),2)^2 - aquad^2);
            xMiddle = beacons(closestBeacon(1),1) + aquad*(beacons(closestBeacon(3),1)-beacons(closestBeacon(1),1))/circDist;
            yMiddle = beacons(closestBeacon(1),2) + aquad*(beacons(closestBeacon(3),2)-beacons(closestBeacon(1),2))/circDist;
            
            lidEstimateCoords(3*2-1, 2) = xMiddle + hquad*(beacons(closestBeacon(3),2)-beacons(closestBeacon(1),2))/circDist;
            lidEstimateCoords(3*2-1, 1) = yMiddle - hquad*(beacons(closestBeacon(3),1)-beacons(closestBeacon(1),1))/circDist;
            lidEstimateCoords(3*2, 2) = xMiddle - hquad*(beacons(closestBeacon(3),2)-beacons(closestBeacon(1),2))/circDist;
            lidEstimateCoords(3*2, 1) = yMiddle + hquad*(beacons(closestBeacon(3),1)-beacons(closestBeacon(1),1))/circDist;
        else
            lidEstimateCoords(3*2-1, 2) = beacons(closestBeacon(1),1) + aquad*(beacons(closestBeacon(3),1)-beacons(closestBeacon(1),1))/circDist;
            lidEstimateCoords(3*2, 2) = lidEstimateCoords(3*2-1, 2);
            lidEstimateCoords(3*2-1, 1) = beacons(closestBeacon(1),2) + aquad*(beacons(closestBeacon(3),2)-beacons(closestBeacon(1),2))/circDist;
            lidEstimateCoords(3*2, 1) = lidEstimateCoords(3*2, 1);
        end
    end
end

lidDist = zeros(12,1);
lidIndex = 0;
for shortLoop1 = 3:6
    for shortLoop2 = 1:2
        lidIndex = lidIndex + 1;
        lidDist(lidIndex) = sqrt((lidEstimateCoords(shortLoop1, 1)-lidEstimateCoords(shortLoop2, 1))^2+(lidEstimateCoords(shortLoop1, 2)-lidEstimateCoords(shortLoop2, 2))^2);
    end
end
for shortLoop1 = 5:6
    for shortLoop2 = 3:4
        lidIndex = lidIndex + 1;
        lidDist(lidIndex) = sqrt((lidEstimateCoords(shortLoop1, 1)-lidEstimateCoords(shortLoop2, 1))^2+(lidEstimateCoords(shortLoop1, 2)-lidEstimateCoords(shortLoop2, 2))^2);
    end
end

[lidDistSmallest, lidDistInd] = min(lidDist);
switch lidDistInd
    case 1
        xhat_post_prev(1) = lidEstimateCoords(1,1);
        xhat_post_prev(2) = lidEstimateCoords(1,2);
    case 2
        xhat_post_prev(1) = lidEstimateCoords(2,1);
        xhat_post_prev(2) = lidEstimateCoords(2,2);
    case 3
        xhat_post_prev(1) = lidEstimateCoords(1,1);
        xhat_post_prev(2) = lidEstimateCoords(1,2);
    case 4
        xhat_post_prev(1) = lidEstimateCoords(2,1);
        xhat_post_prev(2) = lidEstimateCoords(2,2);
    case 5
        xhat_post_prev(1) = lidEstimateCoords(3,1);
        xhat_post_prev(2) = lidEstimateCoords(3,2);
    case 6
        xhat_post_prev(1) = lidEstimateCoords(4,1);
        xhat_post_prev(2) = lidEstimateCoords(4,2);
    case 7
        xhat_post_prev(1) = lidEstimateCoords(3,1);
        xhat_post_prev(2) = lidEstimateCoords(3,2);
    case 8
        xhat_post_prev(1) = lidEstimateCoords(4,1);
        xhat_post_prev(2) = lidEstimateCoords(4,2);
    otherwise
end

velmsg.Angular.Z = 0.3;
velmsg.Linear.X = 0.1;
send(control_pub,velmsg);

for i = 1:total_samples
    if i == 500
        velmsg.Angular.Z = 0;
        velmsg.Linear.X = 0.025;
        send(control_pub,velmsg);
    end
    joint = receive(joint_sub);
    imu = receive(imu_sub);
    odom = receive(odom_sub);
    
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
    
    xhat_priori(1) = xhat_post_prev(1) + dT*velmsg.Linear.X*cos(xhat_post_prev(3));
    xhat_priori(2) = xhat_post_prev(2) + dT*velmsg.Linear.X*sin(xhat_post_prev(3));
    xhat_priori(3) = xhat_post_prev(3) + dT*velmsg.Angular.Z;
    
    F_prev = [1, 0, -dT*velmsg.Linear.X*sin(xhat_post_prev(3));
        0, 1, dT*velmsg.Linear.X*cos(xhat_post_prev(3));
        0, 0, 1];
    
    Q_prev = diag([k*abs(joint.Position(1)), k*abs(joint.Position(2)), 0.1]);
    
    P_priori = F_prev*P_post_prev*transpose(F_prev)+Q_prev;
    
    v = v_prev + imu.LinearAcceleration.X*dT;
    
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
    
    R = diag([0.1, 0.2, 0.4]);
    
    K = P_priori*transpose(H)*inv(H*P_priori*transpose(H)+R);
    
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
grid on
hold off

figure(2)
hold on
plot(1:total_samples, xhat_graph(:,3))
plot(1:total_samples, odom_graph(:,3))
title('Ground Truth orientation versus EKF prediciton')
grid on
hold off

figure(3)
hold on
plot(xhat_graph(:,1), xhat_graph(:,2))
plot(odom_graph(:,1), odom_graph(:,2))
title('Ground Truth position versus EKF prediciton')
grid on
hold off

figure(4)
hold on
plot(1:total_samples, time_stamp(:,3))
title('EKF computation time')
grid on
hold off