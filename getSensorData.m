function [sensor_data, laser_data, odometry_data] = getSensorData(data_file)
    data_type = {'L', 'O'};
    
    fid = fopen(data_file);
    if fid == -1
        fclose(fid);
        error('File %s not found!', data_file);
    end
    
    sensor_data = [];
    idx = 0;
    laser_data = [];
    odometry_data = [];
    
    tline = fgets(fid);
    while ischar(tline)
        idx = idx+1;
        if strcmp(tline(1), data_type{1}) == 1
            readings = str2num(tline(3:end));
            sensor_data(idx).timestamp = readings(end);
            sensor_data(idx).type = data_type{1};
            sensor_data(idx).robot_pos = readings(1:3);
            sensor_data(idx).laser_pos = readings(4:6);
            sensor_data(idx).laser_reading = readings(7:end-1);
            laser_data = [laser_data; readings];
        elseif strcmp(tline(1), data_type{2}) == 1
            readings = str2num(tline(3:end));
            sensor_data(idx).timestamp = readings(end);
            sensor_data(idx).type = data_type{2};
            sensor_data(idx).robot_pos = readings(1:3);
            odometry_data = [odometry_data; readings];
        end
        tline = fgets(fid);
    end
    
    fclose(fid);
end