function [deltaS, deltaTheta] = jointStateCalc(sr, sl, srPrev, slPrev)
% function for calculating the joint state inputs [deltaS deltaTheta] using
% the inputs (sr, sl, srPrev, slPrev)
    r = 0.033;
    b = 0.287;
    
    deltaS = (sl - slPrev + sr - srPrev)*r/2;
    deltaTheta = (sl - slPrev - sr + srPrev)*r/b;
end