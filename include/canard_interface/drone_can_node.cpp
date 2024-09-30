#include "drone_can_node.hpp"
#include "canard_interface.hpp"

void DroneCanNode::start_node(const char *interface_name)
{
    canard_iface_.init(interface_name);

    printf("DroneCanNode started on %s, node ID %d\n", interface_name, canard_iface_.get_node_id());

    send_NodeStatus();

    canard_iface_.process(100);

    /*
      Run the main loop.
     */
    uint64_t next_1hz_service_at = micros64();

    uavcan_protocol_GetNodeInfoRequest req;

    req = {};

    while(get_node_info_client_.request(1,req) == false) {
        printf("Requesting Node Info\n");
        canard_iface_.process(10);
    }
    

    while (true) {
    
        uint64_t ts = micros64();

        if (ts >= next_1hz_service_at) {
            next_1hz_service_at += 1000000ULL;
            send_NodeStatus();
        }

        canard_iface_.process(10);
    }
}

void DroneCanNode::handle_EscStatus(const CanardRxTransfer &transfer, 
const uavcan_equipment_esc_Status &msg)
{

    printf("ESC index: %u\n", msg.esc_index);
    printf("Voltage: %f\n", msg.voltage);
    printf("Current: %f\n", msg.current);
    printf("Temperature: %f\n", msg.temperature);
    printf("ESC RPM: %u\n", msg.rpm);
    printf("Error count: %u\n", msg.error_count);
    printf("*****************************\n");
    rpm_cmd_.rpm.data[0] = 6000;
    rpm_cmd_.rpm.len = 1;

    esc_rpm_pub_.broadcast(rpm_cmd_);
}
void DroneCanNode::handle_GetNodeInfo(const CanardRxTransfer &transfer, 
const uavcan_protocol_GetNodeInfoResponse &rsp)
{
    printf("Got GetNodeInfo response\n");
    printf("ESC name: ");
    for(int i = 0; i < rsp.name.len; i++) {
        printf("%c", rsp.name.data[i]);
    }
    printf("\n");

}
void DroneCanNode::send_NodeStatus()
{

    node_status_msg_.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status_msg_.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status_msg_.sub_mode = 0;
    node_status_msg_.uptime_sec = millis32() / 1000UL;

    node_status_pub_.broadcast(node_status_msg_);

}
