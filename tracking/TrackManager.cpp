#include "TrackManager.h"

TrackManager::TrackManager(std::vector<Pointcloud> clusters) {
  int size = clusters.size();
  for (int i = 0; i < size; i++) {
    addTarget(clusters[i]);
  }
}

TrackManager::TrackManager() {
}

void TrackManager::addTarget(Pointcloud cluster) {
  targets.push_back(TrackTarget(cluster));
}

void TrackManager::associate(int * &assignment, std::vector<Pointcloud> clusters) {
  int targetNum = targets.size();
  int observeNum = clusters.size();
  // const float GATING_DIST = 0.7;
  const float GATING_DIST = 1.5;

  // calc cost matrix
  double* costMatrix = new double[targetNum*observeNum];

  // each row correspond to one observations(i) ; col: targets (j)
  int count = 0;
  for (int j = 0; j < targetNum; j++) {
    for (int i = 0; i < observeNum; i++) {
      Pointcloud predict = targets[j].predict();
      Pointcloud observe = clusters[i];
      point3d cPredict = TrackTarget::getMass(predict);
      point3d cObserve = TrackTarget::getMass(observe);
      float dist = cPredict.distanceXY(cObserve);

      float gating;
      float score;
      if (targets[j].trajectory.size() <= 1) {
        gating = GATING_DIST;
        score = pow(2, -cPredict.distance(cObserve));
      } else {
        gating = targets[j].getAverageSpeed();
        score = calcScore(predict, observe);
      }

      if (dist > gating || fabs(cPredict.z() - cObserve.z()) > 0.5) {
        costMatrix[count] = ASSIGNMENT_INF;
      } else {
        costMatrix[count] = 1 - score;
      }
      count++;
    }
  }

  // log costMatrix
  cout << "costMatrix:" << endl;
  for (int i = 0; i < observeNum; i++) {
    for (int j = 0; j < targetNum; j++) {
      cout << costMatrix[i+j*observeNum] << " ";
    }
    cout << endl;
  }


  double* cost = new double[1];
  assignmentoptimal(assignment, cost, costMatrix, observeNum, targetNum);
  // log assignment
  cout << "assignment:" << endl;
  for (int i = 0; i < observeNum; i++) {
    cout << assignment[i] << endl;
  }
  delete[] costMatrix;
}

float range(Pointcloud p, char dim) {
  int n = p.size();
  float *a = new float[n];
  float max_, min_;

  if (dim == 'x') {
    for (int i = 0; i < n; i++) {
      a[i] = p[i].x();
    }
  } else if(dim == 'y') {
    for (int i = 0; i < n; i++) {
      a[i] = p[i].y();
    }
  }

  max_ = *max_element(a, a+n-1);
  min_ = *min_element(a, a+n-1);
  delete[] a;
  return max_ - min_;
}

void TrackManager::update(std::vector<Pointcloud> clusters) {
  int tNum = targets.size();
  int cNum = clusters.size();
  if (!tNum) {
    for (int i = 0; i < clusters.size(); i++) {
      addTarget(clusters[i]);
    }
    return;
  } else if (cNum) {
    bool * is_target_matched = new bool[tNum];
    int * assignment = new int[cNum];
    // int * assignment;
    for (int i = 0; i < tNum; i++) {
      is_target_matched[i] = false;
    }

    associate(assignment, clusters);

    for (int i = 0; i < cNum; i++) {
      int assignTo = assignment[i];
      if (assignTo < 0) {
        // new observation
        addTarget(clusters[i]);
      } else {
        is_target_matched[assignTo] = true;
        targets[assignTo].update(clusters[i]);
      }
    }

    // remove false alarm or store disappeared target
    for (int i = tNum - 1; i > 0; i--) {
      if (!is_target_matched[i]) {

        if (targets[i].trajectory.size() > 5) {
          disappearedTargets.push_back(targets[i]);
        }

        targets.erase(targets.begin()+i);
      }
    }

    delete[] is_target_matched;
    delete[] assignment;
  }
}

float TrackManager::calcScore(Pointcloud p1, Pointcloud p2) {
  point3d c1 = TrackTarget::getMass(p1);
  point3d c2 = TrackTarget::getMass(p2);

  float width1 = range(p1, 'x');
  float height1 = range(p1, 'y');
  float width2 = range(p2, 'x');
  float height2 = range(p2, 'y');

  float endx = max(c1.x() + width1/2, c2.x() + width2/2);
  float startx = min(c1.x() - width1/2, c2.x() - width2/2);
  float width = width1+width2-(endx-startx);

  float endy = max(c1.y()+height1/2, c2.y()+height2/2);
  float starty = min(c1.y()-height1/2, c2.y()-height2/2);
  float height = height1+height2-(endy-starty);

  if (width<=0||height<=0) {
    // return pow(2, -c1.distance(c2));
    return 0;
  } else {
    float Area = width * height;
    float Area1 = width1 * height1;
    float Area2 = width2 * height2;
    return 2 * Area / (Area1 + Area2);
  }
}

void TrackManager::saveTargets() {
  int n = 1;
  char file[50];

  for (vector<TrackTarget>::iterator it = targets.begin(); it != targets.end(); it++) {
    TrackTarget target = *(it);
    if (target.trajectory.size() < 5) continue;
    disappearedTargets.push_back(target);
  }

  for (vector<TrackTarget>::iterator it = disappearedTargets.begin(); it != disappearedTargets.end(); it++) {
    TrackTarget target = *(it);

    ofstream out;
    sprintf(file, "trajectory%d.csv", n);
    out.open(file);
    int size = target.trajectory.size();
    for (int i = 0; i < size; i++) {
      out << target.trajectory[i].x() << " " << target.trajectory[i].y() << " " << target.trajectory[i].z() << endl;
      // cout << target.trajectory[i].x() << " " << target.trajectory[i].y() << " " << target.trajectory[i].z() << endl;
    }
    out.close();

    sprintf(file, "points%d.csv", n);
    out.open(file);
    Pointcloud points = target.frames.back();
    size = points.size();
    for (int i = 0; i < size; i++) {
      out << points[i].x() << " " << points[i].y() << " " << points[i].z() << endl;
    }
    out.close();
    n++;
  }

  cout << "saved " << n-1 << "targets" << endl;
}
