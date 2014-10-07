function [ particles ] = particle_filter( data_file, map_file, num_particles, numCores )
%MAIN Summary of this function goes here
%   Detailed explanation goes here

%% params
minLog = -700;

% s = RandStream('mt19937ar','Seed',1);
% RandStream.setGlobalStream(s);

occ_folder = './occ_prob_discrete';

%% parallel
if nargin < 4
    %default to 2 cores
    numCores = 2;
end

% Close the pools, if any
if numCores > 1    
    try
        fprintf('Closing any pools...\n');
        matlabpool close; 
    catch ME
        disp(ME.message);
    end
    fprintf('Starting a pool of workers with %d cores\n', numCores);
    matlabpool('local',numCores);
end
%% particle_filter
[map, particles, entropy] = init(map_file, num_particles);
drawnow;
[sensor_data, laser_data, odometry_data] = getSensorData(data_file);
[sensor_params] = estimate_sensor_params();
iter = 1;
% entropy = 0;
got_initial_likelihood = false;
W = ones(1, num_particles)/num_particles;

while length(sensor_data) > 1
%     unique(particles,'rows')
    
    [particles, observation, offset, move, sensor_data] = sample_motion_model_with_map(particles, sensor_data, map);
    
%     unique(particles,'rows')
    if got_initial_likelihood && ~move
        continue;
    end
    
    logW = zeros(1,num_particles);
%     W = ones(1,num_particles);
    xx = particles(:,1) + offset(1);
    yy = particles(:,2) + offset(2);
    tt = particles(:,3) + offset(3);
    prob = map.prob;
    resolution = map.resolution;
    
    pre = 0;
    
    if pre
        for k = 1:num_particles
            x = xx(k);
            y = yy(k);
            theta = tt(k);
            fprintf('particle: %d\n',k);

            rx = round(x);
            ry = round(y);
            rx(rx>map.max_x) = map.max_x;
            rx(rx<map.min_x) = map.min_x;
            ry(ry>map.max_y) = map.max_y;
            ry(ry<map.min_y) = map.min_y;            
            if map.prob(ry,rx) == -1
                logW(k) = -inf;
                continue;
            end

            fname = sprintf('x%d_y%d.mat', rx, ry);
    %         fname = [occ_folder,'/',fname];
            fname = strcat(occ_folder,'/',fname);
            load(fname,'WEIGHT','DIS'); % can't parallel
            degree = floor(radtodeg(theta));
            idx_b = (degree-89):(degree+90);
            idx_b = mod(idx_b, 360) + 1;
            W = WEIGHT(idx_b);
            D = DIS(idx_b);

            tmp = 0;
            parfor b = 1:length(observation)
                [w_angle] = get_angle_weight_pre(observation(b), W{b}, D{b}, sensor_params );
                tmp = tmp + log(w_angle)/3;
            end  
            logW(k) = tmp;
        end        
    else
        parfor k = 1:num_particles
            x = xx(k);
            y = yy(k);
            theta = tt(k);
%             fprintf('particle: %d\n',k);

            tmp = 0;
            for b = 1:length(observation)
                [ occ_prob, X, Y ] = get_occ_prob( x, y, theta, b, prob ); 
                diff = [X-x;Y-y];
                dis = resolution*sqrt(sum(diff.^2));
                [ w_angle ] = get_angle_weight( observation(b), occ_prob, dis , sensor_params); 
                tmp = tmp + log(w_angle);
            end  
            logW(k) = tmp;
        end    
    end
    invalid = isinf(logW);
%     W = zeros(size(logW));
    vlog = logW(~invalid);
%     vlog = vlog - min(vlog) + minLog;
    W = W.^0.8;
    W(~invalid) = W(~invalid).*exp(1*vlog/30);
    W(invalid) = W(invalid).*min(W(~invalid));
    W = W/sum(W(:));

    if sum(W) == 0
        fprintf('all zeros. uniform sampling!!!!!!!\n');
        W = ones(size(W)); 
    end
    
    % update num_particles
    min_num_particles = 1000;
    num_particles = max(min_num_particles, num_particles - 500 );

    [particles, entropy, W] = resample(particles, W, num_particles, entropy, ~got_initial_likelihood);
    
    if ~got_initial_likelihood
        got_initial_likelihood = true;
        W = ones(1, num_particles)/num_particles;
    end
    
    visualize(map, [], particles, iter);
    iter = iter+1;
    
end




end

