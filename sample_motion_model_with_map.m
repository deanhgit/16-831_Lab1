function [prediction, observation, pos_offset, move, sensor_data] = sample_motion_model_with_map(particles, sensor_data, map)
    prediction = [];
    observation = [];
    pos_offset = [];
    move = false;
    N = size(particles, 1);
    wall_threshold = 0.1;
    
    last_laser_reading = [];
    while length(sensor_data) > 1
        last_reading = sensor_data(1);
        if isempty(last_laser_reading) && last_reading.type == 'L'
            last_laser_reading = last_reading;
        end
        current_reading = sensor_data(2);
        action = current_reading.robot_pos - last_reading.robot_pos;
        action = [action(1:2) wrapToPi(action(3))];

        sensor_data = sensor_data(2:end);
        if ~isempty(last_laser_reading) && current_reading.type == 'L'
            observation = current_reading.laser_reading;
            pos_offset = current_reading.laser_pos - current_reading.robot_pos;         
        end
        
        move = norm(action) > 0;
        if move
            dt = current_reading.timestamp - last_reading.timestamp;
            if ~isempty(observation)
                prediction = sample_motion_model(particles, action, dt, 0);
            else
                prediction = sample_motion_model(particles, action, dt, 0);
            end
            grid_level_prediction = round(prediction(:, 1:2));
            pose_consistency = zeros(N, 1);
            valid_grid_idx = find(grid_level_prediction(:,1)>=1 & grid_level_prediction(:,1)<=map.size_x & grid_level_prediction(:,2)>=1 & grid_level_prediction(:,2) <= map.size_y);
            pose_consistency(valid_grid_idx)... 
                = map.prob(sub2ind([map.size_y, map.size_x], grid_level_prediction(valid_grid_idx,2), grid_level_prediction(valid_grid_idx,1)));
            resample_iteration = 0;
            while ~isempty(pose_consistency(pose_consistency <= wall_threshold & pose_consistency >= 0)) && resample_iteration < 500
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
        
        if ~isempty(observation)
            break;
        end
    end
end

function [prediction] = sample_motion_model(particles, action, dt, resolution)
    K = 1;

    A = [3, 1, 3, 0.25];
    a = 3*dt*A;
%     a = 0.5*ones(4, 1);
    N = size(particles, 1);
    X = particles(:,1); Y = particles(:,2); THETA = particles(:,3);

    dx = action(1); dy = action(2); dtheta = action(3);
    d_rot_1 = atan2(dy, dx) - dtheta;
    d_trans = norm([dx, dy]);
    d_rot_2 = dtheta - d_rot_1;
    
    sigmas = [a(1)*d_rot_1.^2+a(2)*d_trans.^2, a(3)*d_trans.^2+a(4)*(d_rot_1.^2+d_rot_2.^2), a(1)*d_rot_2.^2+a(2)*d_trans.^2];
%     sigmas = zeros(4, 1);
    
    dd_rot_1 = d_rot_1 - normrnd(0, sigmas(1), [N, K]);
    dd_trans = d_trans - normrnd(0, sigmas(2), [N, K]);
    dd_rot_2 = d_rot_2 - normrnd(0, sigmas(3), [N, K]);
    
%     prob_dd_rot_1 = normpdf(dd_rot_1-d_rot_1, 0, sigmas(1));
%     prob_dd_trans = normpdf(dd_trans-d_trans, 0, sigmas(2));
%     prob_dd_rot_2 = normpdf(dd_rot_2-d_rot_2, 0, sigmas(3));
    
    X = X + dd_trans.*cos(THETA+dd_rot_1);
    Y = Y + dd_trans.*sin(THETA+dd_rot_1);
    THETA = THETA + dd_rot_1 + dd_rot_2;
    
%     XX = repmat(X, [1, K]) + dd_trans.*cos(repmat(THETA, [1, K])+dd_rot_1);
%     YY = repmat(Y, [1, K]) + dd_trans.*sin(repmat(THETA, [1, K])+dd_rot_1);
%     TT = repmat(THETA, [1, K]) + dd_rot_1 + dd_rot_2;
    
    prediction = [X Y THETA];
end