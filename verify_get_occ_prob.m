function [ occ_prob, X, Y ] = verify_get_occ_prob( x,y, theta,b_idx )
%VERIFY_GET_OCC_PROB Summary of this function goes here
%   Detailed explanation goes here


[map, particles] = init('wean.dat', 0);
hold on;
prob = map.prob;

[ occ_prob, X, Y ] = get_occ_prob( x, y, theta, b_idx, prob );
plot(X,Y);

hold off;



end

