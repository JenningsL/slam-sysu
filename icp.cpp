#include "icp.h"

Matrix<float, 4, Dynamic> getFeaturesMatrix(Pointcloud P) {
  int nPoints = P.size();
  Matrix<float, 4, Dynamic> features;
  features.resize(4, nPoints);

  int col = 0;
  for (Pointcloud::iterator it = P.begin(); it != P.end(); it++) {
    features(0, col) = (*it).x();
    features(1, col) = (*it).y();
    features(2, col) = (*it).z();
    features(3, col) = 1;
    col ++;
  }
  // cout << features << endl;
  return features;
}

Pointcloud toPointCloud(Matrix<float, 4, Dynamic> features) {
  Pointcloud pc;
  int n = features.outerSize();
  for (int i = 0; i < n; i++) {
    float x = features(0, i) / features(3, i);
    float y = features(1, i) / features(3, i);
    float z = features(2, i) / features(3, i);
    pc.push_back(x, y, z);
  }
  return pc;
}

Pointcloud icp(Pointcloud Q, Pointcloud P)
{
  Matrix<float, 4, Dynamic> features_Q = getFeaturesMatrix(Q);
  Matrix<float, 4, Dynamic> features_P = getFeaturesMatrix(P);

  Labels featureLabels;
  featureLabels.push_back(Label("x"));
  featureLabels.push_back(Label("y"));
  featureLabels.push_back(Label("z"));

  DP data(features_P, featureLabels);
  DP ref(features_Q, featureLabels);

  PM::ICP icp;
  // load YAML config
  string configFile = "icp_config.yaml";
  ifstream ifs(configFile.c_str());
  icp.loadFromYaml(ifs);
  // icp.setDefault();
  PM::TransformationParameters T = icp(data, ref);

  // Transform data to express it in ref
  DP data_out(data);
  icp.transformations.apply(data_out, T);

  // Save files to see the results
  // ref.save("test_ref.vtk");
  // data.save("test_data_in.vtk");
  // data_out.save("test_data_out.vtk");
  // cout << T <<endl;

	return toPointCloud(data_out.features);
}
