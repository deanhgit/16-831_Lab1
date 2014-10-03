function [ w_angle ] = get_angle_weight_pre( r, weight, dis , sensor_params )
%GET_ANGLE_WEIGHT_PRE Summary of this function goes here
%   Detailed explanation goes here
% 

toosmall = weight < 0.05;
weight(toosmall) = [];
dis(toosmall) = [];


r_prob = get_reading_prob(r, dis, sensor_params);
w_angle = sum(r_prob.*weight);

% [~,idx] = max(weight);
% r_prob = get_reading_prob(r, dis(idx), sensor_params);
% w_angle = r_prob;

end

