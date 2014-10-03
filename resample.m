function [new_particles, entropy, new_weights] = resample(particles, weights, num_samples, old_entropy)
    if nargin == 2
        num_samples = size(particles, 1);
    end

    weights = weights + min(weights(weights>0));
    weights = weights/sum(weights(:));

%     particles_idx = 1:size(particles, 1);
%     new_samples_idx = randsample(particles_idx, num_samples, true, weights);
%     new_particles = particles(new_samples_idx, :);
    entropy = -mean(log(weights(weights>0)));
%     fprintf('entropy = %d\n', entropy);
    ratio = max(weights(:))/min(weights(:));
    fprintf('max(W)/min(W) = %d\n', ratio);
    if true %ratio < 1e5 %old_entropy > 0 && entropy > 1.05*old_entropy
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
        new_particles = particles;
        new_weights = weights;
    end
end