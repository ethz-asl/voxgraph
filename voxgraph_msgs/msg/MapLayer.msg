# Message header
Header header

# Map header encapsulating the map's main properties
voxgraph_msgs/MapHeader map_header

# Layer containing the map's TSDF and ESDF
# NOTE: It is expressed in the frame specified by header.frame_id
voxblox_msgs/Layer tsdf_layer
voxblox_msgs/Layer esdf_layer
