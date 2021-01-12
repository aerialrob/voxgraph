#include "voxgraph/tools/data_servers/projected_map_server.h"

//#include <cblox/core/tsdf_esdf_submap.h>
#include <cblox_msgs/MapLayer.h>
#include <voxblox_ros/conversions.h>
#include <voxblox/core/layer.h>
#include <voxblox/integrator/merge_integration.h>

namespace voxgraph {
ProjectedMapServer::ProjectedMapServer(ros::NodeHandle nh_private) {
  projected_tsdf_map_pub_ =
      nh_private.advertise<cblox_msgs::MapLayer>("projected_map_tsdf", 1, true);
  projected_esdf_map_pub_ =
      nh_private.advertise<cblox_msgs::MapLayer>("projected_map_esdf", 1, true);
}

void ProjectedMapServer::publishProjectedMap(
    const VoxgraphSubmapCollection& submap_collection,
    const VoxgraphSubmapCollection::ProjectedTsdfMapPtr& collection_tsdf_map,
    const VoxgraphSubmapCollection::ProjectedEsdfMapPtr& collection_esdf_map,
    const ros::Time& timestamp, bool tsdf_layer) {

    if(tsdf_layer){
      // Only publish if there are subscribers
      if (projected_tsdf_map_pub_.getNumSubscribers() > 0) {
        publishProjectedMap(submap_collection, collection_tsdf_map, collection_esdf_map, timestamp, projected_tsdf_map_pub_, tsdf_layer);
      }
    }else{
      // Only publish if there are subscribers
      if (projected_esdf_map_pub_.getNumSubscribers() > 0) {
        publishProjectedMap(submap_collection, collection_tsdf_map, collection_esdf_map, timestamp, projected_esdf_map_pub_, tsdf_layer);
      }
    }
}

void ProjectedMapServer::publishProjectedMap(
    const VoxgraphSubmapCollection& submap_collection,
    const VoxgraphSubmapCollection::ProjectedTsdfMapPtr& collection_tsdf_map,
    const VoxgraphSubmapCollection::ProjectedEsdfMapPtr& collection_esdf_map,
    const ros::Time& timestamp, const ros::Publisher& projected_map_publisher, bool tsdf_layer) {
  // Create the message and set its headers
  cblox_msgs::MapLayer projected_map_tsdf_msg;
  projected_map_tsdf_msg.header = generateHeaderMsg(timestamp);
  projected_map_tsdf_msg.map_header = generateMapHeaderMsg(submap_collection);
  
  std::clock_t timer;
  if(tsdf_layer){
    // Set the message's TSDF
    voxblox::serializeLayerAsMsg<voxblox::TsdfVoxel>(
        collection_tsdf_map->getTsdfLayer(), false,
        &projected_map_tsdf_msg.tsdf_layer);
    double serializeLayerTSDF = (double)(std::clock() - timer) / CLOCKS_PER_SEC;
    projected_map_tsdf_msg.tsdf_layer.action =
        static_cast<uint8_t>(voxblox::MapDerializationAction::kUpdate);

  }
  else{
    // Set the message's ESDF
    // Generate ESDF from TSDF collection
    timer = std::clock();
    Transformation T_identity;

    voxblox::serializeLayerAsMsg<voxblox::EsdfVoxel>(
        collection_esdf_map->getEsdfLayer(), false,
        &projected_map_tsdf_msg.esdf_layer);
    projected_map_tsdf_msg.esdf_layer.action =
        static_cast<uint8_t>(voxblox::MapDerializationAction::kUpdate);
    double serializeLayerESDF = (double)(std::clock() - timer) / CLOCKS_PER_SEC;
    timer = std::clock();
    std::cout << "Serialize ESDF: \n"
              << "----------------------- \n"
              << "serializeLayerESDF " << serializeLayerESDF << "\n";
  }

  // Publish
  projected_map_publisher.publish(projected_map_tsdf_msg);
}

VoxgraphSubmapCollection::ProjectedTsdfMapPtr ProjectedMapServer::getProjectedTsdfMap(const VoxgraphSubmapCollection& submap_collection){
  return submap_collection.getProjectedMap();
}

VoxgraphSubmapCollection::ProjectedEsdfMapPtr ProjectedMapServer::getProjectedEsdfMap(VoxgraphSubmapCollection::ProjectedTsdfMapPtr&  collection_tsdf_map){
  Transformation T_identity;
  VoxgraphSubmap::Ptr projected_map_ptr = std::make_shared<VoxgraphSubmap>(
      T_identity, 1, collection_tsdf_map->getTsdfLayer());
  projected_map_ptr->generateCollectionEsdf();
  return projected_map_ptr->getEsdfMapPtr();
}

void ProjectedMapServer::updateProjectedTsdfMap(VoxgraphSubmapCollection::ProjectedTsdfMapPtr& collection_tsdf_map, VoxgraphSubmap* active_submap){

  voxblox::Layer<voxblox::TsdfVoxel>* combined_tsdf_layer_ptr =
      collection_tsdf_map->getTsdfLayerPtr();

  // Getting the tsdf submap and its pose
  const voxblox::TsdfMap& tsdf_map = active_submap->getTsdfMap();
  const Transformation& T_G_S = active_submap->getPose();

  // Merging layers the submap into the global layer
  voxblox::mergeLayerAintoLayerB(tsdf_map.getTsdfLayer(), T_G_S,
                          combined_tsdf_layer_ptr);
  
}

void ProjectedMapServer::updateProjectedEsdfMap(VoxgraphSubmapCollection::ProjectedEsdfMapPtr& collection_esdf_map, VoxgraphSubmap* active_submap){

  std::clock_t timer;
  timer = std::clock();
  voxblox::Layer<voxblox::EsdfVoxel>* combined_esdf_layer_ptr =
      collection_esdf_map->getEsdfLayerPtr();

  Transformation T_identity;
  const Transformation& T = active_submap->getPose();
  VoxgraphSubmap::Ptr active_map_ptr = std::make_shared<VoxgraphSubmap>(
      T_identity, 1, active_submap->getTsdfMap().getTsdfLayer());
  active_map_ptr->generateCollectionEsdf();

  // Getting the esdf submap and its pose
  const voxblox::EsdfMap& esdf_map = active_map_ptr->getEsdfMap();
  const Transformation& T_G_S = active_submap->getPose();


  // Merging layers the submap into the global layer
   voxblox::mergeLayerAintoLayerB(active_map_ptr->getEsdfMap().getEsdfLayer(),
            T_G_S, combined_esdf_layer_ptr);

  double updateProjectedEsdfMap = (double)(std::clock() - timer) / CLOCKS_PER_SEC;
  std::cout << "updateProjectedEsdfMap : \n"
              << "----------------------- \n"
              << "updateProjectedEsdfMap " << updateProjectedEsdfMap << "\n";
}

std_msgs::Header ProjectedMapServer::generateHeaderMsg(
    const ros::Time& timestamp) {
  std_msgs::Header msg_header;
  // TODO(victorr): Get the world frame name from FrameNames once implemented
  msg_header.frame_id = "mission";
  msg_header.stamp = timestamp;
  return msg_header;
}

cblox_msgs::MapHeader ProjectedMapServer::generateMapHeaderMsg(
    const VoxgraphSubmapCollection& submap_collection) {
  // Set the map ID and type
  cblox_msgs::MapHeader map_header;
  map_header.id = 0;
  map_header.is_submap = false;

  // Set the map's start and end time
  if (!submap_collection.empty()) {
    map_header.start_time =
        submap_collection.getSubmap(submap_collection.getFirstSubmapId())
            .getStartTime();
    map_header.end_time =
        submap_collection.getSubmap(submap_collection.getLastSubmapId())
            .getEndTime();
  } else {
    map_header.start_time = ros::Time(0.0);
    map_header.end_time = ros::Time(0.0);
  }

  // Set the pose estimate to zero
  // TODO(victorr): Get the world frame name from FrameNames once implemented
  map_header.pose_estimate.frame_id = "mission";
  tf::poseKindrToMsg(Transformation().cast<double>(),
                     &map_header.pose_estimate.map_pose);

  return map_header;
}
}  // namespace voxgraph
