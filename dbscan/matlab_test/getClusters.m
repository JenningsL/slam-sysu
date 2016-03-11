function [cluster, centroids] = getClusters( points, IDX )
%GETCLUSTER 根据 dbscan 的聚类结果整理点簇数据和每个点簇的质心
%   {return} cluster: 1*n的cell类型的点簇数组,每个元素为一个点簇的所有点集
%            centroids: 对应每个点簇的质心坐标
pointsNum = size(IDX, 1);
clusterNum = size(unique(IDX), 1) - 1;
cluster_ = cell(1, clusterNum);

for i = 1:pointsNum
    idx = IDX(i);
    if idx ~= 0
    cluster_{idx} = [cluster_{idx} points(i,:)'];
    end
end
%% filter out small cluster
% cluster = cluster_;
cluster = cell(1); % expand automatically
ii = 1;
for j = 1:clusterNum
    if length(cluster_{j}) > 100
        cluster{ii} = cluster_{j};
        ii = ii + 1;
    end
end
%% calculate centroids
validClusterNum = length(cluster);
centroids = zeros(3, validClusterNum);
for k = 1:validClusterNum
    centroids(:,k) = mean(cluster{k}, 2);
end
end

