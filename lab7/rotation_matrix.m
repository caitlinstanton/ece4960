theta         = yaw_7;
theta_radians = deg2rad(theta);
r             = tof_7;
 
x_data = zeros(1, length(r));
y_data = zeros(1, length(r));
 
 
for i = 1:length(r)
    r_matrix = [r(i); 0; 1];
    rotation_matrix = [cos(theta_radians(i)) -sin(theta_radians(i)) 0; sin(theta_radians(i)) cos(theta_radians(i)) 0; 0 0 1;];
    new_matrix = rotation_matrix*r_matrix;
    x_data(i) = (new_matrix(1)-114);
    y_data(i) = (new_matrix(2)-177);
end
 
plot(x_data,y_data)
hold on
 
theta         = yaw_8;
theta_radians = deg2rad(theta);
r             = tof_8;
 
x_data = zeros(1, length(r));
y_data = zeros(1, length(r));
 
 
for i = 1:length(r)
    r_matrix = [r(i); 0; 1];
    rotation_matrix = [cos(theta_radians(i)) -sin(theta_radians(i)) 0; sin(theta_radians(i)) cos(theta_radians(i)) 0; 0 0 1;];
    new_matrix = rotation_matrix*r_matrix;
    x_data(i) = (new_matrix(1)-114);
    y_data(i) = (new_matrix(2)-177);
end
 
plot(x_data,y_data)
hold on
 
theta         = yaw_9;
theta_radians = deg2rad(theta);
r             = tof_9;
 
x_data = zeros(1, length(r));
y_data = zeros(1, length(r));
 
 
for i = 1:length(r)
    r_matrix = [r(i); 0; 1];
    rotation_matrix = [cos(theta_radians(i)) -sin(theta_radians(i)) 0; sin(theta_radians(i)) cos(theta_radians(i)) 0; 0 0 1;];
    new_matrix = rotation_matrix*r_matrix;
    x_data(i) = (new_matrix(1)-114);
    y_data(i) = (new_matrix(2)-177);
end
 
plot(x_data,y_data)
hold on
 
theta         = yaw_10;
theta_radians = deg2rad(theta);
r             = tof_10;
 
x_data = zeros(1, length(r));
y_data = zeros(1, length(r));
 
 
for i = 1:length(r)
    r_matrix = [r(i); 0; 1];
    rotation_matrix = [cos(theta_radians(i)) -sin(theta_radians(i)) 0; sin(theta_radians(i)) cos(theta_radians(i)) 0; 0 0 1;];
    new_matrix = rotation_matrix*r_matrix;
    x_data(i) = (new_matrix(1)-114);
    y_data(i) = (new_matrix(2)-177);
end
 
plot(x_data,y_data)
hold on
 
theta         = yaw_11;
theta_radians = deg2rad(theta);
r             = tof_11;
 
x_data = zeros(1, length(r));
y_data = zeros(1, length(r));
 
 
for i = 1:length(r)
    r_matrix = [r(i); 0; 1];
    rotation_matrix = [cos(theta_radians(i)) -sin(theta_radians(i)) 0; sin(theta_radians(i)) cos(theta_radians(i)) 0; 0 0 1;];
    new_matrix = rotation_matrix*r_matrix;
    x_data(i) = (new_matrix(1)-114);
    y_data(i) = (new_matrix(2)-177);
end
 
plot(x_data,y_data)
hold on
 
theta         = yaw_12;
theta_radians = deg2rad(theta);
r             = tof_12;
 
x_data = zeros(1, length(r));
y_data = zeros(1, length(r));
 
 
for i = 1:length(r)
    r_matrix = [r(i); 0; 1];
    rotation_matrix = [cos(theta_radians(i)) -sin(theta_radians(i)) 0; sin(theta_radians(i)) cos(theta_radians(i)) 0; 0 0 1;];
    new_matrix = rotation_matrix*r_matrix;
    x_data(i) = (new_matrix(1)-114);
    y_data(i) = (new_matrix(2)-177);
end
 
plot(x_data,y_data)
hold on
 
theta         = yaw_13;
theta_radians = deg2rad(theta);
r             = tof_13;
 
x_data = zeros(1, length(r));
y_data = zeros(1, length(r));
 
 
for i = 1:length(r)
    r_matrix = [r(i); 0; 1];
    rotation_matrix = [cos(theta_radians(i)) -sin(theta_radians(i)) 0; sin(theta_radians(i)) cos(theta_radians(i)) 0; 0 0 1;];
    new_matrix = rotation_matrix*r_matrix;
    x_data(i) = (new_matrix(1)-114);
    y_data(i) = (new_matrix(2)-177);
end
 
plot(x_data,y_data)
hold off
