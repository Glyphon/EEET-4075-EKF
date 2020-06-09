function [xHat, yHat, thetaHat] = lidarCalc(xHatPrior, yHatPrior, thetaHatPrior)
    
    beacons = [0, 0; 3, 0; 3, 3; 0, 3]; % Beacon coordinates (i,1)=x (i,2)=y
    phiTolerance = 15; % Tolerance of angle reading in degree integers
    beaconRadius = 0.125; % Radius of beacons
    beaconMeasureOriginal = zeros(4,2); % Angle and distance of beacon from Lidar (i,1)=Phi (i,2)=r
    closestBeacons = zeros(4,1); % Vector showing order of closest to furthest beacons
    beacNums = 0; % Total number of beacons measured
    lidEstimateCoords = zeros(6,2); % List of possible coordinates (i,1)=x (i,2)=y
    lidDist = zeros(12,1); % List of distances between coordinates
    % [11-21, 12-21, 11-22, 12-22, 11-31, 12-31, 11-32, 12-32, 21-31, 22-31, 21-32, 22-32]
    lidIndex = 0; % Index used for lidDist
    lidar = receive(scan_sub);
    
    % Preprocessing data to reject out of range values
    lidar.Ranges(lidar.Ranges == Inf) = NaN;
    lidar.Ranges(lidar.Ranges > lidar.RangeMax) = NaN;
    lidar.Ranges(lidar.Ranges < lidar.RangeMin) = NaN;
    
    % Determine Lidar angle range where beacon 1 should be
    lidar1Rough = atan2((beacons(1,2) - yHatPrior),(beacons(1,1) - xHatPrior)) - thetaHatPrior;
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
    
    % Find angle and radius where beacon 1 is and adjust initial thetaHat
    [lidar1Min, lidar1MinInd] = min(lidar1Ranges);
    beaconMeasureOriginal(1,1) = (lidar1MinInd + lidar1RoughLow - 1)*lidar.AngleIncrement;
    beaconMeasureOriginal(1,2) = lidar1Min + beaconRadius;
    if any(beaconMeasureOriginal(1,2))
        thetaHatPrior = thetaHatPrior + lidar1Rough - beaconMeasureOriginal(1,1);
        if abs(thetaHatPrior) > pi
            if thetaHatPrior > pi
                thetaHatPrior =- 2*pi;
            else
                thetaHatPrior =+ 2*pi;
            end
        end
    end
    
    % Determine Lidar angle range where other beacons should be
    lidar2Rough = atan2((beacons(2,2) - yHatPrior),(beacons(2,1) - xHatPrior)) - thetaHatPrior;
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
    
    lidar3Rough = atan2((beacons(3,2) - yHatPrior),(beacons(3,1) - xHatPrior)) - thetaHatPrior;
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
    
    lidar4Rough = atan2((beacons(4,2) - yHatPrior),(beacons(4,1) - xHatPrior)) - thetaHatPrior;
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
    
    % Find angle of radius of other beacons
    [lidar2Min, lidar2MinInd] = min(lidar2Ranges);
    [lidar3Min, lidar3MinInd] = min(lidar3Ranges);
    [lidar4Min, lidar4MinInd] = min(lidar4Ranges);
    beaconMeasureOriginal(2,1) = (lidar2MinInd + lidar2RoughLow - 1)*lidar.AngleIncrement;
    beaconMeasureOriginal(2,2) = lidar2Min + beaconRadius;
    beaconMeasureOriginal(3,1) = (lidar3MinInd + lidar4RoughLow - 1)*lidar.AngleIncrement;
    beaconMeasureOriginal(3,2) = lidar3Min + beaconRadius;
    beaconMeasureOriginal(4,1) = (lidar4MinInd + lidar5RoughLow - 1)*lidar.AngleIncrement;
    beaconMeasureOriginal(4,2) = lidar4Min + beaconRadius;
    
    % Determine how many, and which, beacons where measured
    beaconMeasure = beaconMeasureOriginal;
    while any(beaconMeasure(:,2))
        beacNums =+ 1;
        [~, closestBeacon(beacNums)] = min(beaconMeasure(:,2));
        beaconMeasure(closestBeacon(beacNums),2) = NaN;
    end
    
    % Determine possible coordinates
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
    
    % Determine distance between possible coordinates
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
    
    % Estimate final position
    
end