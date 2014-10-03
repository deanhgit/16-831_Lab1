function [ prob ] = get_reading_prob( reading, expected , params )
%GET_READING_PROB Summary of this function goes here
%   everything in cm

max_read = params.max_read;
max_read_prob = params.max_read_prob;
sig = params.sigma;
g_w = params.gaussian_weight;
e_w = params.exponential_weight;
lamb = params.lambda;

offset = params.offset;

% expected(expected > max_read) = max_read;

prob = g_w*normpdf(reading,expected,sig) + e_w*lamb*exp(-lamb*reading) + offset;

% if reading >= max_read
%     prob = prob + max_read_prob;
% end



end



