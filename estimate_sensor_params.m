function [ sensor_params ] = estimate_sensor_params()
%GET_SENSOR_PARAM Summary of this function goes here
%   estimate sensor model parameters from the data

% max_read = 500;
max_read = 10000;
fname = './log/robotdata3.log';
max_line = 81;
% max_degree = 18;

%% collect data (robotdata3.log)
% [sensor_data,~,~] = getSensorData(fname);
% R = [];
% for k = 1:max_line
%     d = sensor_data(k);
%     if strcmp(d.type,'L');
%         R = [R; d.laser_reading;];
%     end
% end
% 
% R = R - repmat(R(1,:),[size(R,1),1]);
% R = R(:);


% max_read_prob = 0.1;
% g_w = 0.8;
max_read_prob = 0.001;
g_w = 0.5;
% sigma = 5;
sigma = 100;
% hist(R)
% sigma = std(R)
lambda = 0.0015;
u_w = 0.05;
e_w = 1-max_read_prob-g_w-u_w;

offset = u_w/max_read;



%%
sensor_params.max_read = max_read;
sensor_params.max_read_prob = max_read_prob;

sensor_params.sigma = sigma;
sensor_params.gaussian_weight = g_w;

sensor_params.lambda = lambda;
sensor_params.exponential_weight = e_w;

sensor_params.offset = offset;

x = 1:max_read;
y = zeros(size(x));
r = 0.6*max_read;

y = y + g_w*normpdf(x,r,sigma) + e_w*lambda*exp(-lambda*x) + offset;
y(max_read) = y(max_read)+max_read_prob;

plot(x,y);




end

