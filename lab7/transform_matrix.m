% scan at (12,10) [in]
load('scans.mat')
 
xR = 114.3;
yR = 177.8;
t = time_7*10^-6;    %time vector in seconds
theta = -yaw_7*pi/180;  % angle in rad
distance = tof_7;
x_7 = zeros(1, length(t));  % vector for x values
y_7 = zeros(1, length(t));  % vector for y values
for k=1:length(t)  % for each measurement
    PS = [distance(k); 0; 1];
    robot2world = [cos(theta(k)) -sin(theta(k)) xR;
                   sin(theta(k))  cos(theta(k)) yR;
                   0            0           1 ];
    sensor2robot = [1 0 63.5;
                    0 1 12.7;
                    0 0 1 ];
    PW = robot2world*sensor2robot*PS;
    x_7(k) = PW(1);
    y_7(k) = PW(2);
end
 
xR = 114.3;
yR = 177.8+(1*304.8);
t = time_8*10^-6;    %time vector in seconds
theta = -yaw_8*pi/180;  % angle in rad
distance = tof_8;
x_8 = zeros(1, length(t));  % vector for x values
y_8 = zeros(1, length(t));  % vector for y values
for k=[1:length(t)]  % for each measurement
    PS = [distance(k); 0; 1];
    robot2world = [cos(theta(k)) -sin(theta(k)) xR;
                   sin(theta(k))  cos(theta(k)) yR;
                   0            0           1 ];
    sensor2robot = [1 0 63.5
                    0 1 12.7
                    0 0 1 ];
    PW = robot2world*sensor2robot*PS;
    x_8(k) = PW(1);
    y_8(k) = PW(2);
end
 
xR = 114.3;
yR = 177.8+(4*304.8);
t = time_9*10^-6;    %time vector in seconds
theta = -yaw_9*pi/180;  % angle in rad
distance = tof_9;
x_9 = zeros(1, length(t));  % vector for x values
y_9 = zeros(1, length(t));  % vector for y values
for k=[1:length(t)]  % for each measurement
    PS = [distance(k); 0; 1];
    robot2world = [cos(theta(k)) -sin(theta(k)) xR;
                   sin(theta(k))  cos(theta(k)) yR;
                   0            0           1 ];
    sensor2robot = [1 0 63.5
                    0 1 12.7
                    0 0 1 ];
    PW = robot2world*sensor2robot*PS;
    x_9(k) = PW(1);
    y_9(k) = PW(2);
end
 
xR = 114.3;
yR = 177.8+(7*304.8);
t = time_10*10^-6;    %time vector in seconds
theta = -yaw_10*pi/180;  % angle in rad
distance = tof_10;
x_10 = zeros(1, length(t));  % vector for x values
y_10 = zeros(1, length(t));  % vector for y values
for k=[1:length(t)]  % for each measurement
    PS = [distance(k); 0; 1];
    robot2world = [cos(theta(k)) -sin(theta(k)) xR;
                   sin(theta(k))  cos(theta(k)) yR;
                   0            0           1 ];
    sensor2robot = [1 0 63.5
                    0 1 12.7
                    0 0 1 ];
    PW = robot2world*sensor2robot*PS;
    x_10(k) = PW(1);
    y_10(k) = PW(2);
end
 
xR = 114.3+(7*304.8);
yR = 177.8+(4*304.8);
t = time_11*10^-6;    %time vector in seconds
theta = -yaw_11*pi/180;  % angle in rad
distance = tof_11;
x_11 = zeros(1, length(t));  % vector for x values
y_11 = zeros(1, length(t));  % vector for y values
for k=[1:length(t)]  % for each measurement
    PS = [distance(k); 0; 1];
    robot2world = [cos(theta(k)) -sin(theta(k)) xR;
                   sin(theta(k))  cos(theta(k)) yR;
                   0            0           1 ];
    sensor2robot = [1 0 63.5
                    0 1 12.7
                    0 0 1 ];
    PW = robot2world*sensor2robot*PS;
    x_11(k) = PW(1);
    y_11(k) = PW(2);
end
 
xR = 114.3+(6*304.8);
yR = 177.8+(3*304.8);
t = time_12*10^-6;    %time vector in seconds
theta = -yaw_12*pi/180;  % angle in rad
distance = tof_12;
x_12 = zeros(1, length(t));  % vector for x values
y_12 = zeros(1, length(t));  % vector for y values
for k=[1:length(t)]  % for each measurement
    PS = [distance(k); 0; 1];
    robot2world = [cos(theta(k)) -sin(theta(k)) xR;
                   sin(theta(k))  cos(theta(k)) yR;
                   0            0           1 ];
    sensor2robot = [1 0 63.5
                    0 1 12.7
                    0 0 1 ];
    PW = robot2world*sensor2robot*PS;
    x_12(k) = PW(1);
    y_12(k) = PW(2);
end
 
xR = 114.3+(5*304.8);
yR = 177.8+(1*304.8);
t = time_13*10^-6;    %time vector in seconds
theta = -yaw_13*pi/180;  % angle in rad
distance = tof_13;
x_13 = zeros(1, length(t));  % vector for x values
y_13 = zeros(1, length(t));  % vector for y values
for k=[1:length(t)]  % for each measurement
    PS = [distance(k); 0; 1];
    robot2world = [cos(theta(k)) -sin(theta(k)) xR;
                   sin(theta(k))  cos(theta(k)) yR;
                   0            0           1 ];
    sensor2robot = [1 0 63.5;
                    0 1 12.7;
                    0 0 1 ];
    PW = robot2world*sensor2robot*PS;
    x_13(k) = PW(1);
    y_13(k) = PW(2);
end
 
x0 = 114.3
y0 = 177.8
step = 304.8
plot([x0,x0,x0+3*step,x0+3*step,x0,x0,x0+4*step,x0+4*step,x0+8.5*step,x0+8.5*step,x0+5*step,x0],[y0,y0+2*step,y0+2*step,y0+3*step,y0+3*step,y0+7*step,y0+7*step,y0+5*step,y0+5*step,y0+3*step,y0+step,y0])
hold on
%figure(1)
scatter(x_7,y_7,1)
%figure(2)
hold on
scatter(x_8,y_8,1)
hold on
scatter(x_9,y_9,1)
hold on
scatter(x_10,y_10,1)
hold on
scatter(x_11,y_11,1)
hold on
scatter(x_12,y_12,1)
hold on
scatter(x_13,y_13,1)
hold off
