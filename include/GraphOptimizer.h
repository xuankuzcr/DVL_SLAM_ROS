//
// Created by jongsik on 20. 10. 30..
//

#ifndef DVL_SLAM_MODIFY_GRAPHOPTIMIZER_H
#define DVL_SLAM_MODIFY_GRAPHOPTIMIZER_H

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include "Config.h"

class GraphOptimizer{
public:
  GraphOptimizer(Config &config);
  ~GraphOptimizer();

  void AddEdge();
  void Optimize();

private:
  g2o::SparseOptimizer graphOptimizer;

};


#endif //DVL_SLAM_MODIFY_GRAPHOPTIMIZER_H
