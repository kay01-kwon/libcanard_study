cmake_minimum_required(VERSION 3.10)

# Project Name
project(ESC_Node)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CANARD_INCLUDE /include)


# Compiler flags
add_definitions(-DDRONECAN_CXX_WRAPPERS)

set(CANARD_SRC
    ${CANARD_INCLUDE}/canard_internals/canard.c
    ${CANARD_INCLUDE}/drivers/socketcan.c
)
# DSDL generated sources
set(DSDL_GENERATED_SRC
    include/dsdl_generated/uavcan.protocol.NodeStatus.c
    include/dsdl_generated/uavcan.protocol.GetNodeInfo_res.c
    include/dsdl_generated/uavcan.protocol.GetNodeInfo_req.c
    include/dsdl_generated/uavcan.equipment.esc.RawCommand.c
    include/dsdl_generated/uavcan.equipment.esc.RPMCommand.c
    include/dsdl_generated/uavcan.equipment.esc.Status.c
    include/dsdl_generated/uavcan.protocol.dynamic_node_id.Allocation.c
    include/dsdl_generated/uavcan.protocol.param.GetSet_req.c
    include/dsdl_generated/uavcan.protocol.param.GetSet_res.c
    include/dsdl_generated/uavcan.protocol.param.ExecuteOpcode_req.c
    include/dsdl_generated/uavcan.protocol.param.ExecuteOpcode_res.c
)

# # Include directories
include_directories(
  ${CANARD_INCLUDE}/dsdl_generated
  ${CANARD_INCLUDE}/canard_internals
  ${CANARD_BASE}/drivers
)

add_library(canard STATIC ${CANARD_SRC}) 


add_library(dsdl_generated ${DSDL_GENERATED_SRC} ${CANARD_BASE})

# Target for ESC Node
add_executable(esc_node 
src/esc_node_main.cpp
include/canard_interface/canard_interface.cpp
include/canard_interface/drone_can_node.cpp)
target_link_libraries(esc_node PRIVATE canard dsdl_generated)