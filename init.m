function [map, particles, initial_entropy] = init(map_file, M)
    angle_samples = 1;
    prob_power = 0;

    map = loadMap(map_file);
    prob = map.prob;
    known_grids = find(prob>=0.9);
    sample_prob = prob(known_grids).^prob_power;
    sample_prob = sample_prob/sum(sample_prob(:));
    particles_id = randsample(known_grids, M/angle_samples, true, sample_prob); %% squared prob
    [y, x] = ind2sub([map.size_y, map.size_x], particles_id);
    initial_entropy = -mean(log(prob(particles_id)))*prob_power;
    particles = [repmat([x, y], [angle_samples, 1]) rand(M, 1)*2*pi];
    visualize(map, [], particles);
end

function map = loadMap(filename)
    headers = {'robot_specifications->global_mapsize_x', ...
               'robot_specifications->global_mapsize_y', ...
               'robot_specifications->resolution', ...
               'robot_specifications->autoshifted_x', ...
               'robot_specifications->autoshifted_y', ...
               'global_map[0]:'};
    
    map = [];
    fid = fopen(filename);
    if fid == -1
        fclose(fid);
        error('File %s not found!', filename);
    end
    
    tline = fgets(fid);
    while ischar(tline) && strncmp(tline, headers{6}, length(headers{6})) ~= 1
        if strncmp(tline, headers{3}, length(headers{3}))
            map.resolution = str2double(tline(33:end));
            fprintf('# Map resolution: %d cm\n', map.resolution);
        elseif strncmp(tline, headers{4}, length(headers{4}))
            map.offset_x = str2double(tline(36:end));
            fprintf('# Map offsetX: %g cm\n', map.offset_x);
        elseif strncmp(tline, headers{5}, length(headers{5}))
            map.offset_y = str2double(tline(36:end));
            fprintf('# Map offsetY: %g cm\n', map.offset_y);
        end
        tline = fgets(fid);
    end
    
    [A, ~, ~] = sscanf(tline, 'global_map[0]: %d %d');
    if length(A)~=2
        fclose(fid);
        error('Corrupted file %s!', filename);
    end
%     map.size_x = A(1); map.size_y = A(2);
    map.size_x = A(2); map.size_y = A(1);
    fprintf('# Map size: %d %d\n', map.size_x, map.size_y);
    map.min_x = map.size_x; map.max_x = 0;
    map.min_y = map.size_y; map.max_y = 0;
%     map.prob = zeros(map.size_x, map.size_y);
    map.prob = zeros(map.size_y, map.size_x);
    
    count = 0;
    temp = fscanf(fid,'%e');
    for x = 1:map.size_x
        for y = 1:map.size_y
            
            if temp(count+1) < 0
%                 map.prob(x, y) = -1;
                map.prob(y, x) = -1;
            else
                if x < map.min_x
                    map.min_x = x;
                elseif x > map.max_x
                    map.max_x = x;
                end
                if y < map.min_y
                    map.min_y = y;
                elseif y > map.max_y
                    map.max_y = y;
                end
%                 map.prob(x, y) = temp(count+1);
                map.prob(y, x) = temp(count+1);
            end
            count = count + 1;
        end
    end
    
    fclose(fid);
end