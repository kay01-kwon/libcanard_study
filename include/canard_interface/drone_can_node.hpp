#ifndef DRONE_CAN_NODE_HPP
#define DRONE_CAN_NODE_HPP
// system includes
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <stdbool.h>

// include the canard C++ APIs
#include "canard/publisher.h"
#include "canard/subscriber.h"
#include "canard/service_client.h"
#include "canard/service_server.h"
#include "canard/handler_list.h"
#include "canard/transfer_object.h"

// include the base canard API
#include "canard_internals/canard.h"

// include the interface
#include "driver/socketcan.h"

#include "dsdl_generated/dronecan_msgs.h"
#include "canard_interface.hpp"

static uint64_t micros64()
{
    static uint64_t first_us;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t tus = (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
    if(first_us == 0)
    {
        first_us = tus;
    }

    return tus - first_us;

}

static uint32_t millis32()
{
    return micros64() / 1000ULL;
}

class CanardInterface;

class DroneCanNode
{
    public:

        void start_node(const char *interface_name);

    private:

        CanardInterface canard_iface_{0};
        
        Canard::Publisher<uavcan_protocol_NodeStatus> node_status_pub_{canard_iface_};
        Canard::Publisher<uavcan_equipment_esc_RPMCommand> esc_rpm_pub_{canard_iface_};

        void handle_EscStatus(const CanardRxTransfer& transfer, const uavcan_equipment_esc_Status& msg);
        Canard::ObjCallback<DroneCanNode, uavcan_equipment_esc_Status> esc_status_cb_{this, &DroneCanNode::handle_EscStatus};
        Canard::Subscriber<uavcan_equipment_esc_Status> esc_status_sub_{esc_status_cb_, 0};

        void handle_GetNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoResponse& rsp);
        Canard::ObjCallback<DroneCanNode, uavcan_protocol_GetNodeInfoResponse> get_node_info_cb_{this, &DroneCanNode::handle_GetNodeInfo};
        Canard::Client<uavcan_protocol_GetNodeInfoResponse> get_node_info_client_{canard_iface_, get_node_info_cb_};

        void send_NodeStatus();
        
        uavcan_protocol_NodeStatus node_status_msg_;
        uavcan_equipment_esc_RPMCommand rpm_cmd_;

};

#endif // DRONECAN_NODE_HPP