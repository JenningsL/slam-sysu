data = csvread('cluster.csv');

points = data(:, 1:3);
idx = data(:, 4);
PlotClusterinResult(points, idx);
