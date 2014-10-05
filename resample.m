function [new_particles, entropy, new_weights] = resample(particles, weights, num_samples, old_entropy)
    if nargin == 2
        num_samples = size(particles, 1);
    end

%     new_info_threshold = 1.05;
    new_info_threshold = -(log(0.95/(0.05*num_samples))*0.05 + log(0.05/(0.95*num_samples))*0.95);
    weights = weights + min(weights(weights>0));
    weights = weights/sum(weights(:));

%     particles_idx = 1:size(particles, 1);
%     new_samples_idx = randsample(particles_idx, num_samples, true, weights);
%     new_particles = particles(new_samples_idx, :);
    entropy = -mean(log(weights(weights>0)));
    fprintf('entropy = %d -> %d ', old_entropy, entropy);
%     ratio = max(weights(:))/min(weights(:));
%     fprintf('max(W)/min(W) = %d\n', ratio);
    if entropy > new_info_threshold%*old_entropy
        fprintf('Resampling...');
        new_particles = zeros(size(particles));
        new_weights = zeros(size(weights));
        r = rand/num_samples;
        c = weights(1);
        i = 1;

        for m = 1:num_samples
            U = r+(m-1)/num_samples;
            while U > c
                i = mod(i, num_samples)+1;
                c = c+weights(i);
            end
            new_particles(m,:) = particles(i,:);
            new_weights(m) = weights(i);
        end
        fprintf(' Done\n');
    else
        fprintf('\n');
        new_particles = particles;
        new_weights = weights;
%         entropy = old_entropy;
    end
end