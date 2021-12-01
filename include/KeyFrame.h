//
// Created by jongsik on 20. 10. 30..
//

#ifndef DVL_SLAM_MODIFY_KEYFRAME_H
#define DVL_SLAM_MODIFY_KEYFRAME_H

#include "Config.h"
#include "Frame.h"
#include <thread>
#include <mutex>
#include "PinholeModel.h"

class KeyFrame{
public:
  typedef std::shared_ptr<KeyFrame> Ptr;

  KeyFrame(Config &config, Frame::Ptr frame);
  ~KeyFrame();

  float GetVisibleRatio (const KeyFrame::Ptr keyframe);


  Frame::Ptr frame;

private:
  Config& config_;

  PinholeModel pinholeModel_;

};

class KeyFrameDB {
public:
  typedef std::shared_ptr<KeyFrameDB> Ptr;

  KeyFrameDB(){}
  ~KeyFrameDB(){}
  void Add(KeyFrame::Ptr keyframe) {
    keyframeDB_.push_back(keyframe);
  }

  KeyFrame::Ptr LatestKeyframe() {
    return *(keyframeDB_.end()-1);
  }

  void LatestKeyframe(std::vector<KeyFrame::Ptr>& keyframe_window, int n) {

    for(int i = n; i > 0; --i) {
      keyframe_window.push_back( *(keyframeDB_.end() - i) );
    }

  }

  void KeyframeSet(std::vector<KeyFrame::Ptr>& keyframe_window, int idx, int n) {

    for(int i = n; i > 0; --i) {
      keyframe_window.push_back( keyframeDB_[idx-i] );
    }

  }

  void ConnectedKeyframe(std::vector<KeyFrame::Ptr>& connected_keyframe, int n) {

    for(int i = n; i > 1; --i) {
      connected_keyframe.push_back( *(keyframeDB_.end() - i) );
    }

  }

  int size() {
    return keyframeDB_.size();
  }

  std::vector<KeyFrame::Ptr>::iterator begin() { return keyframeDB_.begin(); }
  std::vector<KeyFrame::Ptr>::iterator end() { return keyframeDB_.end(); }
  std::vector<KeyFrame::Ptr>& keyframeDB() { return keyframeDB_; }

private:
  std::vector<KeyFrame::Ptr> keyframeDB_;
};

#endif //DVL_SLAM_MODIFY_KEYFRAME_H
