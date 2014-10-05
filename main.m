clear;
map_file = 'wean.dat';
data_file = './log/robotdata1.log';
num_particles = 5000;
numCores = 3;
[ particles ] = particle_filter( data_file, map_file, num_particles, numCores );