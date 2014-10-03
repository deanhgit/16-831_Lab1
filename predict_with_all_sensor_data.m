function [prediction, observation, end_idx] = predict_with_all_sensor_data(particles, sensor_data, start_idx, map)
    i = start_idx;
    N = size(particles, 1);
    X = particles(:,1); Y = particles(:,2); THETA = particles(:,3);
    resolution = map.resolution;
    observation = [];
    
    while i < length(sensor_data)
        action = sensor_data(i+1).robot_pos - sensor_data(i).robot_pos;
        translation = norm(action(1:2))/resolution;
        rotation = action(3);
        delta_time = sensor_data(i+1).timestamp - sensor_data(i).timestamp;
%     forward modeling rotation
        sigma_rot = pi/180;
        E_rot = normrnd(0, abs(sigma_rot*rotation), [N, 1]);
        THETA = THETA + rotation + E_rot;
%     forward modeling translation
        sigma_translation = 1;
        sigma_drift = pi/180;

        sigma_trs = sigma_translation*sqrt(delta_time);
        sigma_drft = sigma_drift*sqrt(delta_time/2);

        E_trs = normrnd(0, sigma_trs*translation, [N, 1]);
        E_drft = normrnd(0, sigma_drft*translation, [N, 1]);
        THETA = THETA + E_drft;
        X = X + (translation+E_trs).*cos(THETA);
        Y = Y + (translation+E_trs).*sin(THETA);
        E_drft = normrnd(0, sigma_drft*translation, [N, 1]);
        THETA = THETA + E_drft;        
        
        if sensor_data(i+1).type == 'L'
            observation = sensor_data(i+1).laser_reading;
            break;
        else
            i = i+1;
        end
    end
    
    prediction = [X, Y, THETA];
    end_idx = i+1;
end