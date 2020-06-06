clc
syms x x1 y y1 theta T v omega
f1 = sqrt((y1-(y+T*v*sin(theta)))^2+(x1-(x+T*v*cos(theta)))^2);
f2 = atan((y1-(y+T*v*sin(theta)))/(x1-(x+T*v*cos(theta))));

simplify(diff(f1, x), 'Steps', 50)
simplify(diff(f1, y), 'Steps', 50)
simplify(diff(f1, theta), 'Steps', 50)
simplify(diff(f2, x), 'Steps', 100)
simplify(diff(f2, y), 'Steps', 100)
simplify(diff(f2, theta), 'Steps', 100)