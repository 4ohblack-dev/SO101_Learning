# Source this file in BOTH the Endpoint and topic echo terminals.
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/config/fastdds_udp_only.xml"
