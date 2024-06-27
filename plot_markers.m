function plot_markers(Markers , master)
% This function plots the location of a set of points in 3D space

% It expects as input a set of points with specific location and
% orientation
% Markers.location = location of point
% Markers.orientation = orientation of point
% n = number (int) of master terminal

% Plot settings
axisLength = 1; % Length of the axes to draw

figure;
hold on;

% Plot each midpoint
% for i = 1:size(data, 1)
%     reformatted_data = str2num(cell2mat(data.Var2(i))); % Extract [x, y, z] for the point
%     plot3(reformatted_data(1), reformatted_data(2), reformatted_data(3), 'xr', 'MarkerSize', 8); % Plot with red 'x'
%     hold on
% end

% Plot marker center and orientation
for i = 1:length(Markers)
    % Location of markers
    x = Markers(i).location(1);
    y = Markers(i).location(2);
    z = Markers(i).location(3);

    % Plot marker center
    if i==master
        text(x+.2, y+.2, z+.2, num2str(i), 'FontSize', 12, 'HorizontalAlignment', 'center','Color','r'); % Print number at the marker position
    else
        text(x+.2, y+.2, z+.2, num2str(i), 'FontSize', 12, 'HorizontalAlignment', 'center'); % Print number at the marker position
    end
    % Draw orientation axes
    quiver3(x, y, z, ...
        Markers(i).orientation(1, 1)*axisLength, Markers(i).orientation(1, 2)*axisLength, Markers(i).orientation(1, 3)*axisLength, ...
        'r', 'LineWidth', 2); % Red for the x axis
    quiver3(x, y, z, ...
        Markers(i).orientation(2, 1)*axisLength, Markers(i).orientation(2, 2)*axisLength, Markers(i).orientation(2, 3)*axisLength, ...
        'g', 'LineWidth', 2); % Green for the y axis
    quiver3(x, y, z, ...
        Markers(i).orientation(3, 1)*axisLength, Markers(i).orientation(3, 2)*axisLength, Markers(i).orientation(3, 3)*axisLength, ...
        'b', 'LineWidth', 2); % Blue for the z axis
end

grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;
set(gca, 'YDir','reverse')
view(3);
hold off;
drawnow;

end