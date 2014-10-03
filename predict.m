% http://www.cim.mcgill.ca/~yiannis/particletutorial.pdf

function [prediction, observation, pos_offset, move] = predict(particles, laser_data, map)
    N = size(particles, 1);    
    X = particles(:,1); Y = particles(:,2); THETA = particles(:,3);
    resolution = map.resolution;
    
    MULT = 1;
    
    last_reading = laser_data(1,:);
    current_reading = laser_data(2,:);
    observation = current_reading(7:end-1);
    action = current_reading(1:3)-last_reading(1:3);
    pos_offset = current_reading(4:6) - current_reading(1:3); %% change
    translation = norm(action(1:2))/resolution;
    rotation = action(3);
    if translation > 0 || rotation ~= 0 || 1
        move = 1; %% change
        delta_time = current_reading(end) - last_reading(end);

    %     forward modeling rotation
        sigma_rot = pi/180 * MULT;
        E_rot = normrnd(0, abs(sigma_rot*rotation), [N, 1]);
        THETA = THETA + rotation + E_rot;

    %     forward modeling translation
        sigma_translation = 1 * MULT;
        sigma_drift = pi/180 * MULT;

        sigma_trs = sigma_translation*sqrt(delta_time);
        sigma_drft = sigma_drift*sqrt(delta_time/2);

        E_trs = normrnd(0, sigma_trs*translation, [N, 1]);
        E_drft = normrnd(0, sigma_drft*translation, [N, 1]);
        THETA = THETA + E_drft;
        X = X + (translation+E_trs).*cos(THETA);
        Y = Y + (translation+E_trs).*sin(THETA);
        E_drft = normrnd(0, sigma_drft*translation, [N, 1]);
        THETA = THETA + E_drft;

        prediction = [X, Y, THETA];    
    else 
        prediction = particles;
        move = 0;
    end
end