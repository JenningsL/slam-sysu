% data = csvread('cluster.csv');
% 
% points = data(:, 1:3);
% idx = data(:, 4);
% PlotClusterinResult(points, idx);

p = csvread('../../cluster.csv');
plot3(p(:,1),p(:,2),p(:,3),'b.');
axis equal;
