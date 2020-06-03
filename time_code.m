clear variables, close all
clc

rosshutdown;
rosinit('http://10.128.0.216:11311');

control_sub = rossubscriber('/cmd_vel');
joint_sub = rossubscriber('/joint_states');
imu_sub = rossubscriber('/imu');
pause(2)

total_samples = 1000;

time_graph_joint = zeros(total_samples,2);
time_graph_imu = zeros(total_samples,2);

for i = 1:total_samples
    joint = receive(joint_sub);
    imu = receive(imu_sub);
    
    time_graph_joint(i,1) = joint.Header.Stamp.Sec;
    time_graph_joint(i,2) = joint.Header.Stamp.Nsec;
    
    time_graph_imu(i,1) = imu.Header.Stamp.Sec;
    time_graph_imu(i,2) = imu.Header.Stamp.Nsec;
end

rosshutdown;

save('jointStateTime.mat', 'time_diff_joint');
save('imuTime.mat', 'time_diff_imu');

time_diff_joint = zeros(total_samples - 1, 1);
for i = 2:total_samples
    time_diff_joint(i-1) = time_graph_joint(i,1) + time_graph_joint(i,2)*10^-9 - time_graph_joint(i-1,1) - time_graph_joint(i-1,2)*10^-9;
end

time_diff_imu = zeros(total_samples - 1, 1);
for i = 2:total_samples
    time_diff_imu(i-1) = time_graph_imu(i,1) + time_graph_imu(i,2)*10^-9 - time_graph_imu(i-1,1) - time_graph_imu(i-1,2)*10^-9;
end

average_joint = sum(time_diff_joint)/numel(time_diff_joint);
average_imu = sum(time_diff_imu)/numel(time_diff_imu);

figure(1)
hold on
plot(1:total_samples - 1, time_diff_joint)
plot([1 total_samples - 1], [average_joint average_joint])
grid on
hold off

figure(2)
hold on
plot(1:total_samples - 1, time_diff_imu)
plot([1 total_samples - 1], [average_imu average_imu])
grid on
hold off