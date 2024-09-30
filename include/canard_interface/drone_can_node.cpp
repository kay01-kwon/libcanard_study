#include "drone_can_node.hpp"
#include "canard_interface.hpp"

void DroneCanNode::start_node(const char *interface_name)
{
    canard_iface_.init(interface_name);

    printf("DroneCanNode started on %s, node ID %d\n", interface_name, canard_iface_.get_node_id());

    canard_iface_.process(100);

    /*
      Run the main loop.
     */
    uint64_t next_1hz_service_at = micros64();
    // uint64_t next_50hz_service_at = micros64();

    uavcan_protocol_GetNodeInfoRequest req;

    req = {};

    while(get_node_info_client.request(1,req) == false) {
        printf("Requesting Node Info\n");
        canard_iface.process(10);
    }
    

    while (true) {
    
        uint64_t ts = micros64();

        if (ts >= next_1hz_service_at) {
            next_1hz_service_at += 1000000ULL;
            process1HzTasks(ts);
        }
        // if (ts >= next_50hz_service_at) {
        //     next_50hz_service_at += 1000000ULL/100U;
        //     // rpm_pub.broadcast(rpm_cmd);
        // //     // printf("Broadcasting RPM: %d\n", rpm);
        // }
        canard_iface_.process(10);
    }
}
