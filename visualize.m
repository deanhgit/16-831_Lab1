function visualize(map, robot_pos, particles, frame_num)
    if nargin < 2
        robot_pos = [];
        particles = [];
    elseif nargin == 3
        frame_num = 0;
    end
    
%     plot map
    map_fig = findobj('type', 'figure', 'name', 'map');
    if isempty(map_fig)
        figure('name', 'map');
        map_fig = findobj('type', 'figure', 'name', 'map');
    end
    figure(map_fig);
    clf;
    mat = map.prob;
    mat(mat<0) = 0;
    imagesc(mat);
    colormap('gray');
    axis([map.min_y map.max_y map.min_x map.max_x]);
    axis tight
    xlabel('x'); ylabel('y');
    set(gca, 'YDir', 'normal')

    hold on
%     plot robot
    if ~isempty(robot_pos)
        drawRobot(robot_pos, 3)
    end
    
%     plot particles
    if ~isempty(particles)
        plot(particles(:,1), particles(:,2), 'r.', 'MarkerSize', 5);
%         length = 10;
%         for i = 1:size(particles, 1)
%             plot([particles(i, 1), particles(i, 1)+length*cos(particles(i, 3))], [particles(i, 2), particles(i, 2)+length*sin(particles(i, 3))], 'b');
%         end
    end
    
    hold off
    drawnow
    saveas(figure(findobj('type', 'figure', 'name', 'map')), sprintf('frame1000/frame_%03d', frame_num), 'png');
end

function drawRobot(robot_pos, radius)
    xc = robot_pos(1); yc = robot_pos(2);
    ori = robot_pos(3);
%     draw a circle that represents the robot
    angles = 1:360;
    x = xc + radius*cosd(angles);
    y = yc + radius*sind(angles);
%     draw line segment indicating orientation
    xo = xc + radius*cos(ori);
    yo = yc + radius*sin(ori);
    plot(x, y, 'b', [xc, xo], [yc, yo], 'g', xo, yo, 'r.');
    text(xc+radius, yc, sprintf('(%g, %g, %g)', xc, yc, ori), 'Color', 'r');
end