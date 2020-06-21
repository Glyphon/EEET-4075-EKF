function [xHat, yHat, thetaHat] =...
    lidarCalc(xHatPrior, yHatPrior, thetaHatPrior,...
    lidarRanges, lidarRangeMax, lidarRangeMin, lidarAngleIncrement)
    % A function that takes the outputs from a ROS LiDAR and returns
    % an absolute pose estimate based off four beacons.
    
    beacons = [-0.125, -0.125;...
        3.625, -0.125;...
        3.625, 3.125;...
        -0.125, 3.125]; % Beacon coordinates (i,1)=x (i,2)=y
    phiTolerance = 15; % Tolerance of angle reading in degree integers
    beaconRadius = 0.11; % Radius of beacons 0.125 = sim, 0.11 = phys
    lidarOffset = 0.064;
    lidarShift = 0;
    
    yHatPrior = yHatPrior - sin(thetaHatPrior)*lidarOffset;
    xHatPrior = xHatPrior - cos(thetaHatPrior)*lidarOffset;
    
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
    lidarBottRough =  atan2((beaconBottom(2) - yHatPrior),...
        (beaconBottom(1) - xHatPrior)) - thetaHatPrior;
    if lidarBottRough < 0
        lidarBottRough = lidarBottRough + 2*pi;
    end
    lidarBottRough = ceil(lidarBottRough/lidarAngleIncrement);
    if lidarBottRough - phiTolerance <= 0
        lidarShift = phiTolerance + 1 - lidarBottRough;
        lidarRanges = circshift(lidarRanges, lidarShift);
        lidarBottRough = lidarBottRough + lidarShift;
    elseif lidarBottRough + phiTolerance > 360
        lidarShift = 360 - lidarBottRough - phiTolerance;
        lidarRanges = circshift(lidarRanges, lidarShift);
        lidarBottRough = lidarBottRough + lidarShift;
    end
    lidarBottRange = lidarRanges(lidarBottRough-...
        phiTolerance:lidarBottRough+phiTolerance);
    
    % Determine actual angle and range
    [lidarBottDist, lidarBottExact] = min(lidarBottRange);
    lidarBottRough = lidarBottRough - lidarShift;
    lidarBottExact = lidarBottRough - phiTolerance -...
        lidarShift + lidarBottExact;
    lidarBottDist = lidarBottDist + beaconRadius;
    
    % Estimate thetaHat
    thetaHatPrior = thetaHatPrior - (lidarBottExact -...
        lidarBottRough)*lidarAngleIncrement;
    thetaHat = thetaHatPrior + 0.015;

    % Determine Lidar angle range where top beacon should be
    lidarTopRough = atan2((beaconTop(2) - yHatPrior),(beaconTop(1) -...
        xHatPrior)) - thetaHat;
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
    lidarTopRange = lidarRanges(lidarTopRough-...
        phiTolerance:lidarTopRough+phiTolerance);
    
    % Determine actual angle and range
    [lidarTopDist, ~] = min(lidarTopRange);
    lidarTopDist = lidarTopDist + beaconRadius;
    
    % Determine y and x position
    yHatTest = -(lidarBottDist^2 - lidarTopDist^2 -...
        beaconBottom(2)^2 + beaconTop(2)^2)/...
        (2*(beaconBottom(2) - beaconTop(2))) + sin(thetaHat)*lidarOffset;
    if isnan(yHatTest) || abs(yHatTest - yHatPrior) > 0.07
        yHat = yHatPrior;
        xHat = xHatPrior;
    else
        yHat = yHatTest;
        beta1 = (lidarBottDist + lidarTopDist + beaconBottom(2) -...
            beaconTop(2))*(lidarBottDist + lidarTopDist -...
            beaconBottom(2) + beaconTop(2));
        beta2 = (lidarBottDist - lidarTopDist + beaconBottom(2) -...
            beaconTop(2))*(-lidarBottDist + lidarTopDist +...
            beaconBottom(2) - beaconTop(2));
        if xHatPrior > 1.75
            xHat = beaconBottom(1) + sqrt(beta1*beta2)/...
                (2*(beaconBottom(2) - beaconTop(2))) +...
                cos(thetaHat)*lidarOffset;
        else
            xHat = beaconBottom(1) - sqrt(beta1*beta2)/...
                (2*(beaconBottom(2) - beaconTop(2))) +...
                cos(thetaHat)*lidarOffset;
        end
    end

end