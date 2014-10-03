function [ occ_prob, X, Y ] = get_occ_prob( x, y, theta, b_idx, prob_map )
%GET_BEAM Summary of this function goes here
%   prob_map: 1 = empty, 0 = occupied (cant walk)

[max_y, max_x] = size(prob_map);
dis = 1;

% get beam;
X = 1:max_x;
Y = zeros(size(X)); % degree 0 half line
theta_b = theta - pi/2 + degtorad(b_idx); % compute direction relative to world coordinate

rot_mat = [cos(theta_b) -sin(theta_b); sin(theta_b) cos(theta_b)]; % rotate
tmp = rot_mat*[X;Y];

X = tmp(1,:) + x; % translate
Y = tmp(2,:) + y;

if dis
    X = round(X);
    Y = round(Y);    
end

vx = (X >= 1) & (X <= max_x); % inside map only
vy = (Y >= 1) & (Y <= max_y);
valid = vx & vy;
X = X(valid);
Y = Y(valid);

if dis % lookup prob (discretize);
    idx = sub2ind(size(prob_map), Y, X);
    occ_prob = prob_map(idx);
else % lookup prob (interp2)
    occ_prob = interp2(prob_map,X,Y);
end

end

