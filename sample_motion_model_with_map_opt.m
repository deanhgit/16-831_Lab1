function [prediction, observation, pos_offset, move, sensor_data] = sample_motion_model_with_map_opt(particles, sensor_data, map)
    prediction = [];
    observation = [];
    pos_offset = [];
    move = false;
    N = size(particles, 1);
    
    last_laser_reading = [];
    while length(sensor_data) > 1
        last_reading = sensor_data(1);
        if isempty(last_laser_reading) && last_reading.type == 'L'
            last_laser_reading = last_reading;
        end
        current_reading = sensor_data(2);
        action = current_reading.robot_pos - last_reading.robot_pos;
        action = [action(1:2)/map.resolution wrapToPi(action(3))];
        
        if ~isempty(last_laser_reading) && current_reading.type == 'L'
            observation = current_reading.laser_reading;
        end
        
        move = norm(action) > 0;
        if move
            dt = current_reading.timestamp - last_reading.timestamp;
            prediction = sample_motion_model(particles, action, dt);
            grid_level_prediction = round(prediction(:, 1:2));
            pose_consistency = zeros(N, 1);
            valid_grid_idx = find(grid_level_prediction(:,1)>=1 & grid_level_prediction(:,1)<=map.size_x & grid_level_prediction(:,2)>=1 & grid_level_prediction(:,2) <= map.size_y);
            pose_consistency(valid_grid_idx)... 
                = map.prob(sub2ind([map.size_y, map.size_x], grid_level_prediction(valid_grid_idx,2), grid_level_prediction(valid_grid_idx,1)));
            resample_iteration = 0;
            while ~isempty(pose_consistency(pose_consistency <= 0)) && resample_iteration < 500
                inconsistent_idx = find(pose_consistency <= 0);
                prediction(inconsistent_idx, :) = sample_motion_model(particles(inconsistent_idx, :), action, dt);
                grid_level_prediction = round(prediction(:, 1:2));
                pose_consistency = zeros(N, 1);
                valid_grid_idx = find(grid_level_prediction(:,1)>=1 & grid_level_prediction(:,1)<=map.size_x & grid_level_prediction(:,2)>=1 & grid_level_prediction(:,2) <= map.size_y);
                pose_consistency(valid_grid_idx)... 
                    = map.prob(sub2ind([map.size_y, map.size_x], grid_level_prediction(valid_grid_idx,2), grid_level_prediction(valid_grid_idx,1)));
                resample_iteration = resample_iteration + 1;
            end
            if resample_iteration == 500
                inconsistent_idx = find(pose_consistency <= 0);
                prediction(inconsistent_idx, :) = particles(inconsistent_idx, :);
            end
        else
            prediction = particles;
        end
        
        sensor_data = sensor_data(2:end);
        if ~isempty(observation)
            pos_offset = current_reading.laser_pos - last_laser_reading.laser_pos;
            break;            
        end
    end
end

function [prediction] = sample_motion_model(particles, action, dt)
    A = [5, 5, 5, 5];
    a = sqrt(dt)*A;
    N = size(particles, 1);
    X = particles(:,1); Y = particles(:,2); THETA = particles(:,3);

    dx = action(1); dy = action(2); dtheta = action(3);
    d_rot_1 = atan2(dy, dx) - dtheta;
    d_trans = norm([dx, dy]);
    d_rot_2 = dtheta - d_rot_1;
    
    dd_rot_1 = d_rot_1 - normrnd(0, a(1)*d_rot_1.^2+a(2)*d_trans.^2, [N, 1]);
    dd_trans = d_trans - normrnd(0, a(3)*d_trans.^2+a(4)*(d_rot_1.^2+d_rot_2.^2), [N, 1]);
    dd_rot_2 = d_rot_2 - normrnd(0, a(1)*d_rot_2.^2+a(2)*d_trans.^2, [N, 1]);
    
    X = X + dd_trans.*cos(THETA+dd_rot_1);
    Y = Y + dd_trans.*sin(THETA+dd_rot_1);
    THETA = THETA + dd_rot_1 + dd_rot_2;
    
    prediction = [X Y THETA];
end