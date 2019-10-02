data_x = load('data/identification_x');
data_y = load('data/identification_y');
data_z = load('data/identification_z');
data_yaw = load('data/identification_yaw');


index_x = 0;
index_y = 0;
index_z = 0;
index_yaw = 0;


for ii = size(data_x.input,2):-1:1
    if data_x.input(1,ii) ~= 0
        index_x = ii;
        break;
    end
end

for ii = size(data_y.input,2):-1:1
    if data_y.input(ii) ~= 0
        index_y = ii;
        break;
    end
end

for ii = size(data_z.input,2):-1:1
    if data_z.input(ii) ~= 0
        index_z = ii;
        break;
    end
end

for ii = size(data_yaw.input,2):-1:1
    if data_yaw.input(ii) ~= 0
        index_yaw = ii;
        break;
    end
end

input = data_x.input(1,1:index_x);
time = data_x.time(1,1:index_x) - data_x.time(1,1);
output_z = data_x.output_z(1,1:index_x);
output_x = data_x.output_x(1,1:index_x);
output_y = data_x.output_y(1,1:index_x);
save identification_x.mat input time output_x  output_y output_z


input = data_y.input(1,1:index_y);
time = data_y.time(1,1:index_y) - data_y.time(1,1);
output_z = data_y.output_z(1,1:index_y);
output_x = data_y.output_x(1,1:index_y);
output_y = data_y.output_y(1,1:index_y);
save identification_y.mat input time output_x  output_y output_z

input = data_z.input(1,1:index_z);
time = data_z.time(1,1:index_z) - data_z.time(1,1);
output_z = data_z.output_z(1,1:index_z);
output_x = data_z.output_x(1,1:index_z);
output_y = data_z.output_y(1,1:index_z);
save identification_z.mat input time output_x  output_y output_z

input = data_yaw.input(1,1:index_yaw);
time = data_yaw.time(1,1:index_yaw) - data_yaw.time(1,1);
output_z = data_yaw.output_z(1,1:index_yaw);
output_x = data_yaw.output_x(1,1:index_yaw);
output_y = data_yaw.output_y(1,1:index_yaw);
save identification_yaw.mat input time output_x  output_y output_z