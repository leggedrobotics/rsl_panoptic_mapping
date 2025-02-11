//
// Created by peyschen on 15/03/23.
//
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "nodelet/nodelet.h"
#include "message_matching/MessageMatcher.h"

class MessageMatchingNodelet : public nodelet::Nodelet{
 public:
  MessageMatchingNodelet(){};
  ~MessageMatchingNodelet(){};
 private:
  void onInit() override{
    pnh_ = getPrivateNodeHandle();

    messageMatcher_ = std::make_unique<MessageMatcher>();
    messageMatcher_->setup(pnh_);
  };

  ros::NodeHandle pnh_;
  std::unique_ptr<MessageMatcher> messageMatcher_;
};

PLUGINLIB_EXPORT_CLASS(MessageMatchingNodelet, nodelet::Nodelet);