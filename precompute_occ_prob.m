function [ occ_prob ] = precompute_occ_prob( map, folder )
%PRECOMPUTE_OCC_PROB Summary of this function goes here
%   Detailed explanation goes here
prob_map = map.prob;
% [max_y, max_x] = size(prob_map);
min_x = map.min_x;
max_x = map.max_x;
min_y = map.min_y;
max_y = map.max_y;

% min_x = map.min_y; %% @@
% max_x = map.max_y;
% min_y = map.min_x;
% max_y = map.max_x;

discrete = 1;
x_start = min_x;

for x = x_start:max_x
    for y = min_y:max_y
        
        fname = sprintf('x%d_y%d.mat', x, y);
        fprintf([fname,'\n']);
        
        if map.prob(y,x) == -1;
            fprintf('unknown prob => pass\n')
            continue;
        end
        
        WEIGHT = cell(1,360);
        DIS = cell(1,360);        
        for b = 1:360
            X = 1:max_x;
            Y = zeros(size(X)); % degree 0 half line
            theta_b = degtorad(b);
            rot_mat = [cos(theta_b) -sin(theta_b); sin(theta_b) cos(theta_b)]; % rotate
            tmp = rot_mat*[X;Y];
            X = tmp(1,:) + x; % translate
            Y = tmp(2,:) + y;
            if discrete
                X = round(X);
                Y = round(Y);    
            end
            
            vx = (X >= min_x) & (X <= max_x); % inside map only
            vy = (Y >= min_y) & (Y <= max_y);
            valid = vx & vy;
            X = X(valid);
            Y = Y(valid); 
            
            if discrete % lookup prob (discretize);
                idx = sub2ind(size(prob_map), Y, X);
                occ_prob = prob_map(idx);
            else % lookup prob (interp2)
                occ_prob = interp2(prob_map,X,Y);
            end
            diff = [X-x;Y-y];
            dis = map.resolution*sqrt(sum(diff.^2));   
            
            cut_idx = find(occ_prob <= 0 , 1 ); % nearest unknown or wall (occ_prob == 0)
            pass_prob = occ_prob(1:cut_idx);
            dis = dis(1:cut_idx);
            if ~isempty(cut_idx) && (cut_idx(end) < 0)
                pass_prob(end) = [];
                dis(end) = [];
            end
            block_prob = 1-pass_prob;
            cum_pass = cumprod(pass_prob);
            cum_pass = [1,cum_pass(1:end-1)];
            weight = cum_pass.*block_prob; 
            
            WEIGHT{b} = weight;
            DIS{b} = dis;
            if length(dis) ~= length(weight)
                fprintf('lengths dont match');
            end
        end

        fname = [folder,'/',fname];
        save(fname,'WEIGHT','DIS'); 
    end
end


end

