//
// Created by lorenzo on 2/2/23.
//
#include <panoptic_gridmap/SemanticMap.h>
#include <pcl_ros/transforms.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/bind.hpp>

namespace panoptic_gridmap {
SemanticMap::SemanticMap(ros::NodeHandle& nh, const std::string& configYamlDir) : nh_(nh) {
  // Load parameters from the yaml file
  loadParameters(configYamlDir);
  // crete the grid_map using the information contained in mapConfig_ yaml node
  map_ = grid_map::GridMap({"elevation", "semantic_fused", "things", "stuff", "traversability_cost", "occupancy_semantic"});
  map_.setFrameId(config_["map"]["frame_id"].as<std::string>());
  ROS_INFO_STREAM("map frame id: " << map_.getFrameId());
  map_.setGeometry(grid_map::Length(config_["map"]["width"].as<double>(), config_["map"]["height"].as<double>()),
                   config_["map"]["resolution"].as<double>(),
                   grid_map::Position(config_["map"]["origin"]["x"].as<double>(), config_["map"]["origin"]["y"].as<double>()));
  // init elevation layer to zero
  map_["elevation"].setConstant(0.0);
  map_["stuff"].setConstant(labelToId_["unknown"]);
  // stuff layer is initialized to id corresponding to unknown (last one)
  map_["traversability_cost"].setConstant(std::numeric_limits<double>::quiet_NaN());
  //  create the publisher semanticMapPublisher_ = nh_.advertise<grid_map_msgs::GridMap>(mapConfig_["map_topic"].as<std::string>(), 1,
  //  true);
  // create the subscriber
  std::string dynamicPclTopic = config_["topics"]["dynamic_pcl_topic_sub"].as<std::string>();
  // subscribe subDynamicPcl_ (message_filters::Subscriber<sensor_msgs::PointCloud2>) to topic dynamicPclTopic
  subDynamicPcl_.subscribe(nh_, dynamicPclTopic, 1);
  std::string staticPclTopic = config_["topics"]["static_pcl_topic_sub"].as<std::string>();
  subStaticPcl_.subscribe(nh_, staticPclTopic, 1);
  // sync
  pclExactSyncPtr_.reset(new message_filters::Synchronizer<PclExactSyncPolicy>(PclExactSyncPolicy(10), subDynamicPcl_, subStaticPcl_));
  pclExactSyncPtr_->registerCallback(boost::bind(&SemanticMap::mapCallback, this, _1, _2));
  // publisher
  std::string semanticMapTopic = config_["topics"]["map_topic_pub"].as<std::string>();
  semanticMapPublisher_ = nh_.advertise<grid_map_msgs::GridMap>(semanticMapTopic, 1, true);
  textMarkerPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("semantic_map_text_markers", 1, true);
}

SemanticMap::~SemanticMap() {}

void SemanticMap::loadParameters(const std::string& configYamlDir) {
  //  YAML::Node mapConfig_;
  std::string configYamlPath;
  std::string movablePclTopic;
  try {
    configYamlPath = configYamlDir + "/config.yaml";
    config_ = YAML::LoadFile(configYamlPath);
  } catch (const YAML::BadFile& e) {
    std::cerr << "Error: Failed to load YAML file: " << e.what() << std::endl;
    // print path
    std::cerr << "Path: " << configYamlPath << std::endl;
  } catch (const YAML::BadConversion& e) {
    std::cerr << "Error: Failed to load YAML file: " << e.what() << " check yaml key entries" << std::endl;
  }
  ROS_INFO_STREAM("Loaded config file: " << configYamlPath);
  // load grid map parameters
  // load color_to_label.yaml into the semanticLabelDictionary_
  ROS_INFO_STREAM("Loading color_to_label file: " << configYamlDir);
  std::string colorToLabelYamlPath;
  YAML::Node yamlColorToLabel_;
  nh_.param("panoptic_gridmap/using_ids", usingIds_, false);
  try {
    //   if (!usingIds_) {
    //     colorToLabelYamlPath = configYamlDir + "/color_to_label.yaml";
    //   } else {
    colorToLabelYamlPath = configYamlDir + "/id_to_label.yaml";
    // }
    yamlColorToLabel_ = YAML::LoadFile(colorToLabelYamlPath);
  } catch (const YAML::BadFile& e) {
    std::cerr << "Error: Failed to load YAML file: " << e.what() << std::endl;
    // print path
    std::cerr << "Path: " << colorToLabelYamlPath << std::endl;
  } catch (const YAML::TypedBadConversion<std::string>& e) {
    std::cerr << "Error: Failed to load YAML file: " << e.what() << " check yaml key entries" << std::endl;
  }
  ROS_INFO_STREAM("Loaded color_to_label file: " << colorToLabelYamlPath);
  // print all entries of yamlColorToLabel_ node
  for (YAML::const_iterator it = yamlColorToLabel_.begin(); it != yamlColorToLabel_.end(); ++it) {
    ROS_INFO_STREAM("key: " << it->first.as<std::string>() << " value: " << it->second.as<std::string>());
    idToLabel_[std::stoi(it->first.as<std::string>())] = it->second.as<std::string>();
  }
  ROS_INFO_STREAM("Loaded semanticLabelDictionary_ with " << idToLabel_.size() << " entries");
  // create the inverse dictionary
  for (auto it = idToLabel_.begin(); it != idToLabel_.end(); ++it) {
    labelToId_[it->second] = it->first;
  }
  // idToLabelDictionary_ is the rese
  // load label_to_cost.yaml into the semanticCostDictionary_, some of the costs are marked as inf in the yaml file and should be
  //    converted
  // to std::numeric_limits<float>::infinity()
  std::string labelToCostYamlPath;
  YAML::Node yamlLabelToCost_;
  try {
    labelToCostYamlPath = configYamlDir + "/label_to_cost.yaml";
    yamlLabelToCost_ = YAML::LoadFile(labelToCostYamlPath);
  } catch (const YAML::BadFile& e) {
    std::cerr << "Error: Failed to load YAML file: " << e.what() << std::endl;
    // print path
    std::cerr << "Path: " << labelToCostYamlPath << std::endl;
  } catch (const YAML::TypedBadConversion<std::string>& e) {
    std::cerr << "Error: Failed to load YAML file: " << e.what() << std::endl;
  }
  ROS_INFO_STREAM("Loaded label_to_cost file: " << labelToCostYamlPath);
  labelToCostDictionary_.clear();

  for (const auto& it : yamlLabelToCost_) {
    std::string label = it.first.as<std::string>();
    std::string costStr = it.second.as<std::string>();

    float cost = 0;
    if (costStr != "inf") {
      cost = std::stof(costStr);
    } else {
      cost = std::numeric_limits<float>::infinity();
      ROS_INFO_STREAM("cost for label " << label << " is " << costStr);
    }
    labelToCostDictionary_[label] = cost;
    // load the param using_ids
  }
  //   create the semanticToOccupancyDictionary_ from the semanticCostDictionary_
  //   if the value is inf, then the occupancy is 1 else 0
  for (const auto& it : yamlLabelToCost_) {
    std::string label = it.first.as<std::string>();
    std::string costStr = it.second.as<std::string>();
    int occupancy;
    if (costStr != "inf") {
      occupancy = 0;
    } else {
      occupancy = 1;
      ROS_INFO_STREAM("occupancy for label " << label << " is 1");
    }
    labelToOccupancyDictionary_[label] = occupancy;
  }
}

void SemanticMap::mapCallback(const sensor_msgs::PointCloud2ConstPtr& dynamicPclMsg, const sensor_msgs::PointCloud2ConstPtr& staticPclMsg) {
  // Convert the sensor_msgs::PointCloud2 to a pcl::PointCloud<pcl::PointXYZRGB>
  pcl::PointCloud<pcl::PointXYZRGB> dynamicPcl;
  pcl::fromROSMsg(*dynamicPclMsg, dynamicPcl);
  pcl::PointCloud<pcl::PointXYZRGB> staticPcl;
  pcl::fromROSMsg(*staticPclMsg, staticPcl);
  // transform the pointCloud from the sensor frame to the map frame
  pcl::PointCloud<pcl::PointXYZRGB> transformedPointCloudDynamic;
  // print target frame
  pcl_ros::transformPointCloud(map_.getFrameId(), dynamicPcl, transformedPointCloudDynamic, tfListener_);
  pcl::PointCloud<pcl::PointXYZRGB> transformedPointCloudStatic;
  // print target frame
  pcl_ros::transformPointCloud(map_.getFrameId(), staticPcl, transformedPointCloudStatic, tfListener_);

  // update the semantic, traversability and occupancy layers for each point in the pointCloud
  // we create a temporary grid map to store the new data
  // if multiple points fall in the same cell, the one with the highest cost is kept
  grid_map::GridMap tempMap({"elevation", "things", "stuff", "traversability_cost_things", "traversability_cost_stuff"});
  // initialize traversability_cost and occupancy_semantic with 0
  // same geometry as the map_
  tempMap.setGeometry(map_.getLength(), map_.getResolution(), map_.getPosition());
  // frame
  tempMap.setFrameId(map_.getFrameId());

  // Update the map with the pointCloud
  //  cleanMap(transformedPointCloudStatic, map_);
  updateSingleScanStuffMap(transformedPointCloudStatic, tempMap);
  updateSingleScanThingsMap(transformedPointCloudDynamic, tempMap);

  std::unique_lock<std::mutex> lock(map_mutex_);
  std::vector<std::string> copyLayers = {"stuff"};
  map_.addDataFrom(tempMap, true, true, false, copyLayers);
  fuseLayers(map_, tempMap);
  // overwrite the values of the map_ with the values of the tempMap
  lock.unlock();
  ros::Time time = dynamicPclMsg->header.stamp;
  // Publish the map
  publishSemanticMap(time);
  publishTextForOccupiedCells();
}

std::string SemanticMap::getLabel(int id, std::string type) {
  // the r channel of the point is the index of the label in the semanticLabelDictionary_
  int index = id;
  if (type == "things") {
    index = parseThingsId(index);
  }
  // the label is the value of the index in the semanticLabelDictionary_ (<int, std::string>)
  // throw an error if the index is not in the dictionary
  if (idToLabel_.find(index) == idToLabel_.end()) {
    // print point color
    ROS_ERROR_STREAM("index: " << index << " not found in semanticLabelDictionary_");
  }
  std::string label = idToLabel_[index];
  return label;
}

int SemanticMap::parseThingsId(int id) {
  // id 255 if the cluster has not been assigned a label
  // id 254 if the majority of the cluster has not label
  // id 0 if the cluster id is not in the list of filtered objects (found in dynamic_mapping/config/config.yaml)
  // all of these cases are treated as unknown object -> i.e 0
  // for stuff we keep the label from the output of the semantic segmentation
  // so we have two sources of unknonw objects, 255 and 254 from clustering and 0 from the segmentation
  if (id == 255 || id == 254) {
    id = 0;
  }
  return id;
}

int SemanticMap::parseStuffId(int id) {
  // not assigned labels get mapped to the unknown label id
  int unknownLabelId = labelToId_["unknown"];
  if (id > unknownLabelId) {
    id = unknownLabelId;
  }
  return id;
}

float SemanticMap::getCost(const std::string& label) {
  // the cost is the value of the label in the semanticCostDictionary_ (<std::string, float>)
  float cost = labelToCostDictionary_[label];
  return cost;
}

int SemanticMap::getOccupancy(const std::string& label) {
  // the occupancy is the value of the label in the semanticOccupancyDictionary_ (<std::string, int>)
  int occupancy = labelToOccupancyDictionary_[label];
  return occupancy;
}

void SemanticMap::updateSingleScanThingsMap(const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud, grid_map::GridMap& tempMap) {
  for (int i = 0; i < pointCloud.points.size(); i++) {
    grid_map::Position position(pointCloud.points[i].x, pointCloud.points[i].y);
    if (tempMap.isInside(position)) {
      grid_map::Index index;
      tempMap.getIndex(position, index);
      // calculate the cost based on RGB values
      int labelId = parseThingsId(pointCloud.points[i].r);
      std::string label = getLabel(labelId, "things");
      // if the index matches the index of the obstacle, then print the label
      float cost = getCost(label);
      // if it is not empty, then check if the cost is higher than the one in the tempMap
      // default cost is 0 or nan
      if (cost > tempMap.at("traversability_cost_things", index) || std::isnan(tempMap.at("traversability_cost_things", index))) {
        // if it is higher, then update the semantic layer using the r channel of the point and exclude uncertain objects
        tempMap.at("things", index) = labelId;
        // update the traversability cost layer, with inf
        tempMap.at("traversability_cost_things", index) = cost;
        // update the occupancy layer
      }
    }
  }
}

void SemanticMap::updateSingleScanStuffMap(const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud, grid_map::GridMap& tempMap) {
  for (int i = 0; i < pointCloud.points.size(); i++) {
    grid_map::Position position(pointCloud.points[i].x, pointCloud.points[i].y);
    if (tempMap.isInside(position)) {
      grid_map::Index index;
      tempMap.getIndex(position, index);
      int labelId = parseStuffId(pointCloud.points[i].r);
      // calculate the cost based on RGB values
      std::string label = getLabel(labelId, "stuff");
      // if the index matches the index of the obstacle, then print the label
      float cost = getCost(label);
      // if it is not empty, then check if the cost is higher than the one in the tempMap
      // default cost is 0 or nan
      if (cost > tempMap.at("traversability_cost_stuff", index) || std::isnan(tempMap.at("traversability_cost_stuff", index))) {
        // if it is higher, then update the semantic layer using the r channel of the point and exclude uncertain objects
        if (label != "unknown" and label != "unknownObj") {
          // if more than one point falls inside the same cell, then unknown ones are ignored
          // also not adding the unknown ground allows to just overwrite the ground layer in the permanent map
          tempMap.at("stuff", index) = labelId;
          // update the traversability cost layer
          //          tempMap.at("traversability_cost", index) = cost;
          // update the occupancy layer
          //          tempMap.at("occupancy_semantic", index) = getOccupancy(label);
        }
      }
    }
  }
}

/*
 * Fuse the things and stuff layer of the map into the semantic_fused layer.
 * Things can be on top of stuff (such as ground) and ovveride the stuff label.
 */
void SemanticMap::fuseLayers(grid_map::GridMap& map, grid_map::GridMap& tempMap) {
  map["things"] = tempMap["things"];
  map["semantic_fused"] = map["stuff"];
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index index(*iterator);
    // if not nan and not unknown
    if (!std::isnan(map.at("things", index))) {
      int thingId = map.at("things", index);
      map.at("semantic_fused", index) = thingId;
    }
    // get label for current cell
    std::string label = getLabel(map.at("semantic_fused", index), "stuff");
    float cost = getCost(label);
    map.at("traversability_cost", index) = cost;
    map.at("occupancy_semantic", index) = getOccupancy(label);
  }
}

// void SemanticMap::cleanMap(const pcl::PointCloud<pcl::PointXYZRGB>& pointCloud, grid_map::GridMap& map) {
//   for (int i = 0; i < pointCloud.points.size(); i++) {
//     grid_map::Position position(pointCloud.points[i].x, pointCloud.points[i].y);
//     if (map.isInside(position)) {
//       grid_map::Index index;
//       map.getIndex(position, index);
//       // calculate the cost based on RGB values
//       std::string label = getLabel(pointCloud.points[i]);
//       // if the index matches the index of the obstacle, then print the label
//       float cost = getCost(label);
//       // if it is not empty, then check if the cost is higher than the one in the tempMap
//       // default cost is 0 or nan
//       if (pointCloud.points[i].r == 255 && map.at("occupancy_semantic", index) == 1 || std::isnan(map.at("traversability_cost", index)))
//       {
//         map.at("semantic", index) = 0;
//         // update the traversability cost layer
//         map.at("traversability_cost", index) = cost;
//         // update the occupancy layer
//         map.at("occupancy_semantic", index) = getOccupancy(label);
//       }
//     }
//   }
// }

void SemanticMap::publishTextForOccupiedCells() {
  // iterate over the map layer occupancy and publish a text marker for each occupied cell with the label of the cell
  // get the indices of the occupied cells
  std::vector<grid_map::Index> indices;
  for (grid_map::GridMapIterator iterator(map_); !iterator.isPastEnd(); ++iterator) {
    if (map_.at("occupancy_semantic", *iterator) == 1) {
      indices.push_back(*iterator);
    }
  }
  // get the positions of the occupied cells
  std::vector<grid_map::Position> positions;
  for (int i = 0; i < indices.size(); i++) {
    grid_map::Position position;
    map_.getPosition(indices[i], position);
    positions.push_back(position);
  }
  // get the labels of the occupied cells
  std::vector<std::string> labels;
  for (int i = 0; i < indices.size(); i++) {
    std::string label = idToLabel_[map_.at("semantic_fused", indices[i])];
    labels.push_back(label);
  }
  // publish the text markers
  publishTextGridMapMarkers(positions, labels);
}

void SemanticMap::publishTextGridMapMarkers(std::vector<grid_map::Position> positions, std::vector<std::string> texts) {
  visualization_msgs::MarkerArray markerArray;
  for (int i = 0; i < positions.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_.getFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = "text_labels";
    marker.id = i;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = positions[i].x();
    marker.pose.position.y = positions[i].y();
    marker.pose.position.z = 0.5;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.text = texts[i];
    markerArray.markers.push_back(marker);
  }
  textMarkerPublisher_.publish(markerArray);
}

void SemanticMap::publishSemanticMap(ros::Time time) {
  // Lock the map mutex before getting the map.
  grid_map_msgs::GridMap mapMessage_;
  std::unique_lock<std::mutex> lock(map_mutex_);
  // Code to publish the map message.
  map_.setTimestamp(time.toNSec());
  grid_map::GridMapRosConverter::toMessage(map_, mapMessage_);
  // Unlock the map mutex after getting the map.
  lock.unlock();
  semanticMapPublisher_.publish(mapMessage_);
}
}  // namespace panoptic_gridmap