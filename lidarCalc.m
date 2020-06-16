function [xHat, yHat, thetaHat] = lidarCalc(xHatPrior, yHatPrior, thetaHatPrior, lidarRanges, lidarRangeMax, lidarRangeMin, lidarAngleIncrement)
    
    beacons = [-0.125, -0.125; 3.625, -0.125; 3.625, 3.125; -0.125, 3.125]; % Beacon coordinates (i,1)=x (i,2)=y
    phiTolerance = 30; % Tolerance of angle reading in degree integers
    beaconRadius = 0.125; % Radius of beacons
    lidarShift = 0;
    diag = 0;
    
    % Preprocessing data to reject out of range values
    lidarRanges(lidarRanges == Inf) = NaN;
    lidarRanges(lidarRanges > lidarRangeMax) = NaN;
    lidarRanges(lidarRanges < lidarRangeMin) = NaN;
    
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
    if diag == 1
        lidarBottRough
    end
    if lidarBottRough < 0
        lidarBottRough = lidarBottRough + 2*pi;
    end
    if diag == 1
        lidarBottRough
    end
    lidarBottRough = ceil(lidarBottRough/lidarAngleIncrement);
    if diag == 1
        lidarBottRough
    end
    if lidarBottRough - phiTolerance <= 0
        lidarShift = phiTolerance + 1 - lidarBottRough;
        lidarRanges = circshift(lidarRanges, lidarShift);
        lidarBottRough = lidarBottRough + lidarShift;
    elseif lidarBottRough + phiTolerance > 360
        lidarShift = 360 - lidarBottRough - phiTolerance;
        lidarRanges = circshift(lidarRanges, lidarShift);
        lidarBottRough = lidarBottRough + lidarShift;
    end
    lidarBottRange = lidarRanges(lidarBottRough-phiTolerance:lidarBottRough+phiTolerance);
    if diag == 1
        lidarBottRange
    end
    
    % Determine actual angle and range
    [lidarBottDist, lidarBottExact] = min(lidarBottRange);
    lidarBottRough = lidarBottRough - lidarShift;
    lidarBottExact = lidarBottRough - phiTolerance - lidarShift + lidarBottExact;
    lidarBottDist = lidarBottDist + beaconRadius;
    if diag == 1
        lidarBottExact
        lidarBottDist
    end
    
    % Estimate thetaHat
    thetaHat = thetaHatPrior - (lidarBottExact - lidarBottRough)*lidarAngleIncrement;
    if diag == 1
        thetaHat
    end
    lidarShift = 0;
    % Determine Lidar angle range where top beacon should be
    lidarTopRough = atan2((beaconTop(2) - yHatPrior),(beaconTop(1) - xHatPrior)) - thetaHat;
    if lidarTopRough < 0
        lidarTopRough = lidarTopRough + 2*pi;
    end
    lidarTopRough = ceil(lidarTopRough/lidarAngleIncrement);
    if lidarTopRough - phiTolerance <= 0
        lidarShift = phiTolerance + 1 - lidarTopRough;
        lidarRanges = circshift(lidarRanges, lidarShift);
        lidarTopRough = lidarTopRough + lidarShift;
    elseif lidarTopRough + phiTolerance > 360
        lidarShift = 360 - lidarTopRough - phiTolerance;
        lidarRanges = circshift(lidarRanges, lidarShift);
        lidarTopRough = lidarTopRough + lidarShift;
    end
    lidarTopRange = lidarRanges(lidarTopRough-phiTolerance:lidarTopRough+phiTolerance);
    if diag == 1
        lidarTopRough
        lidarTopRange
    end
    
    % Determine actual angle and range
    [lidarTopDist, ~] = min(lidarTopRange);
    lidarTopDist = lidarTopDist + beaconRadius;
    if diag == 1
        lidarTopDist
    end
    
    % Determine y and x position
    yHat = -(lidarBottDist^2 - lidarTopDist^2 - beaconBottom(2)^2 + beaconTop(2)^2)/(2*(beaconBottom(2) - beaconTop(2)));
    beta1 = (lidarBottDist + lidarTopDist + beaconBottom(2) - beaconTop(2))*(lidarBottDist + lidarTopDist - beaconBottom(2) + beaconTop(2));
    beta2 = (lidarBottDist - lidarTopDist + beaconBottom(2) - beaconTop(2))*(-lidarBottDist + lidarTopDist + beaconBottom(2) - beaconTop(2));
    if xHatPrior > 1.75
        xHat = beaconBottom(1) + sqrt(beta1*beta2)/(2*(beaconBottom(2) - beaconTop(2)));
    else
        xHat = beaconBottom(1) - sqrt(beta1*beta2)/(2*(beaconBottom(2) - beaconTop(2)));
    end
    if diag == 1
        yHat
        beta1
        beta2
        xHat
    end
end