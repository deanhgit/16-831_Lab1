clear;
x=42;
y=420;
theta=pi/2;
b_idx=50;
r = 200;

[map, particles] = init('wean.dat', 0);
hold on;
prob = map.prob;
[ occ_prob, X, Y ] = get_occ_prob( x, y, theta, b_idx, prob );
plot(X,Y);
hold off;
diff = [X-x;Y-y];
dis = map.resolution*sqrt(sum(diff.^2));

[ sensor_params ] = estimate_sensor_params();

[ w_angle ] = get_angle_weight( r, occ_prob, dis , sensor_params);