function [xHat, yHat, thetaHat] = lidarCalc(xHatPrior, yHatPrior, thetaHatPrior)
    
    beacons = [0, 0; 3, 0; 3, 3; 0, 3]; % Beacon coordinates (i,1)=x (i,2)=y
    phiTolerance = 15; % Tolerance of angle reading in degree integers
    beaconRadius = 0.125; % Radius of beacons
    lidar = receive(scan_sub);
    
    % Preprocessing data to reject out of range values
    lidar.Ranges(lidar.Ranges == Inf) = NaN;
    lidar.Ranges(lidar.Ranges > lidar.RangeMax) = NaN;
    lidar.Ranges(lidar.Ranges < lidar.RangeMin) = NaN;
    
    % Determine which half of the court the robot is in
    if xHatPrior > 1.75
        beaconBottom = beacons(2,:);
        beaconTop = beacons(3,:);
    else
        beaconBottom = beacons(1,:);
        beaconTop = beacons(4,:);
    end
    
    % Determine Lidar angle range where bottom beacon should be
    lidarBottRough = atan2((beaconBottom(2) - yHatPrior),(beaconBottom(1) - xHatPrior)) - thetaHatPrior;
    if lidarBottRough < 0
        lidarBottRough =+ 2*pi;
    end
    lidarBottRough = ceil(lidarBottRough/lidar.angle_increment);
    if lidarBottRough - phiTolerance <= 0
        lidarShift = phiTolerance + 1 - lidarBottRough;
        lidar.Ranges = circshift(lidar.Ranges, lidarShift);
        lidarBottRough = lidarBottRough + lidarShift;
    elseif lidarBottRough + phiTolerance > 360
        lidarShift = 360 - lidarBottRough - phiTolerance;
        lidar.Ranges = circshift(lidar.Ranges, lidarShift);
        lidarBottRough = lidarBottRough + lidarShift;
    end
    lidarBottRange = lidar.Ranges(lidarBottRough-phiTolerance:lidarBottRough+phiTolerance);
    
    % Determine actual angle and range
    [lidarBottExact, lidarBottDist] = min(lidarBottRange);
    lidarBottRough = lidarBottRough - lidarShift;
    lidarBottExact = lidarBottRough - phiTolerance - lidarShift + lidarBottExact;
    
    % Estimate thetaHat
    thetaHat = thetaHatPrior - (lidarBottRough - lidarBottExact)*lidar.angle_increment;
    
    % Determine Lidar angle range where top beacon should be
    lidarTopRough = atan2((beaconTop(2) - yHatPrior),(beaconTop(1) - xHatPrior)) - thetaHat;
    if lidarTopRough < 0
        lidarTopRough =+ 2*pi;
    end
    lidarTopRough = ceil(lidarTopRough/lidar.angle_increment);
    if lidarTopRough - phiTolerance <= 0
        lidarShift = phiTolerance + 1 - lidarTopRough;
        lidar.Ranges = circshift(lidar.Ranges, lidarShift);
        lidarTopRough = lidarTopRough + lidarShift;
    elseif lidarTopRough + phiTolerance > 360
        lidarShift = 360 - lidarTopRough - phiTolerance;
        lidar.Ranges = circshift(lidar.Ranges, lidarShift);
        lidarTopRough = lidarTopRough + lidarShift;
    end
    lidarTopRange = lidar.Ranges(lidarTopRough-phiTolerance:lidarTopRough+phiTolerance);
    
    % Determine actual angle and range
    [~, lidarTopDist] = min(lidarTopRange);
    
    % Determine y position
    yHat = -(lidarBottDist^2 - lidarTopDist^2 - beaconBottom(2)^2 + beaconTop(2)^2)/(2*(beaconBottom(2) - beaconTop(2)));
    beta1 = (lidarBottDist + lidarTopDist)^2 - (beaconBottom(2) - beaconTop(2))^2;
    beta2 = (beaconBottom(2) - beaconTop(2))^2 - (lidarBottDist - lidarTopDist)^2;
    if xHatPrior > 1.75
        xHat = beaconBottom(1) - sqrt(beta1*beta2)/(2*(beaconBottom(2) - beaconTop(2)));
    else
        xHat = sqrt(beta1*beta2)/(2*(beaconBottom(2) - beaconTop(2))) + beaconBottom(1);
    end
end