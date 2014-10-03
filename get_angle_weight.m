function [ w_angle ] = get_angle_weight( r, occ_prob, dis , sensor_params)
%GET_ANGLE_WEIGHT Summary of this function goes here
%   dis = numGrid * resolution, numGrid = sqrt(X^2 + Y^2)

cut_idx = find(occ_prob <= 0 , 1 ); % nearest unknown or wall (occ_prob == 0)
pass_prob = occ_prob(1:cut_idx);
dis = dis(1:cut_idx);
if ~isempty(cut_idx) && (cut_idx(end) < 0)
    pass_prob(end) = [];
    dis(end) = [];
end
% occ_prob
block_prob = 1-pass_prob;
cum_pass = cumprod(pass_prob);
cum_pass = [1,cum_pass(1:end-1)];
weight = cum_pass.*block_prob;


[~, idx]=sort(weight,'descend');
K = 2;
K = min(K,length(weight));
weight = weight(idx(1:K))/sum(weight(idx(1:K)));
dis = dis(idx(1:K));

r_prob = get_reading_prob(r, dis, sensor_params);
w_angle = sum(r_prob.*weight);






end

