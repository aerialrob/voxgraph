#ifndef VOXGRAPH_TOOLS_DATA_SERVERS_PROJECTED_MAP_SERVER_H_
#define VOXGRAPH_TOOLS_DATA_SERVERS_PROJECTED_MAP_SERVER_H_

#include <cblox_msgs/MapHeader.h>

#include "voxgraph/common.h"
#include "voxgraph/frontend/submap_collection/voxgraph_submap_collection.h"

namespace voxgraph {
class ProjectedMapServer {
 public:
  explicit ProjectedMapServer(ros::NodeHandle nh_private);

  // Publish the map using this map server instance's ros publisher member
  void publishProjectedMap(
      const VoxgraphSubmapCollection& submap_collection,
      const VoxgraphSubmapCollection::ProjectedTsdfMapPtr& collection_tsdf_map,
      const VoxgraphSubmapCollection::ProjectedEsdfMapPtr& collection_esdf_map,
      const ros::Time& timestamp, bool tsdf_layer);

  // "Bring your own publisher" method
  // NOTE: This method is provided s.t. it can be called using publishers to
  //       custom topics and without requiring a ProjectedMapServer instance.
  //       It is therefore static.
  static void publishProjectedMap(
      const VoxgraphSubmapCollection& submap_collection,
      const VoxgraphSubmapCollection::ProjectedTsdfMapPtr& collection_tsdf_map,
      const VoxgraphSubmapCollection::ProjectedEsdfMapPtr& collection_esdf_map,
      const ros::Time& timestamp, const ros::Publisher& projected_map_publisher,
      bool tsdf_layer);

  VoxgraphSubmapCollection::ProjectedTsdfMapPtr getProjectedTsdfMap(
      const VoxgraphSubmapCollection& submap_collection);
  VoxgraphSubmapCollection::ProjectedEsdfMapPtr getProjectedEsdfMap(
      VoxgraphSubmapCollection::ProjectedTsdfMapPtr& collection_tsdf_map);

  void updateProjectedTsdfMap(
      VoxgraphSubmapCollection::ProjectedTsdfMapPtr& collection_tsdf_map,
      VoxgraphSubmap* active_submap);
  void updateProjectedEsdfMap(
      VoxgraphSubmapCollection::ProjectedEsdfMapPtr& collection_esdf_map, 
      VoxgraphSubmap* active_submap);

 private:
  ros::Publisher projected_tsdf_map_pub_;
  ros::Publisher projected_esdf_map_pub_;
  //TsdfEsdfSubmap::Ptr submap_ptr_;
  // Convenience methods to generate the message and submap headers
  static std_msgs::Header generateHeaderMsg(const ros::Time& timestamp);
  static cblox_msgs::MapHeader generateMapHeaderMsg(
      const VoxgraphSubmapCollection& submap_collection);
};
}  // namespace voxgraph

#endif  // VOXGRAPH_TOOLS_DATA_SERVERS_PROJECTED_MAP_SERVER_H_
