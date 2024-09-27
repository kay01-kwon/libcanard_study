/*
  A simple example DroneCAN node implementing a 4-in-1 ESC using the C++ API

  This example implements 5 features:

   - announces on the bus using NodeStatus at 1Hz
   - answers GetNodeInfo requests
   - implements dynamic node allocation
   - listens for ESC RawCommand commands and extracts throttle levels
   - sends ESC Status messages (with synthetic data based on throttles)
   - a parameter server for reading and writing node parameters

  This example uses socketcan on Linux for CAN transport

  Example usage: ./esc_node vcan0
*/
/*
 This example application is distributed under the terms of CC0 (public domain dedication).
 More info: https://creativecommons.org/publicdomain/zero/1.0/
*/

// system includes
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <stdbool.h>

// include the canard C++ APIs
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <canard/service_client.h>
#include <canard/service_server.h>
#include <canard/handler_list.h>
#include <canard/transfer_object.h>

// include the base canard API
#include <canard.h>

// we are using the socketcan driver
#include <socketcan.h>

// include the headers for the generated DroneCAN messages from the
// dronecan_dsdlc compiler
#include <dronecan_msgs.h>

/*
  in this example we will use dynamic node allocation if MY_NODE_ID is zero
 */
#define MY_NODE_ID 127

/*
  our preferred node ID if nobody else has it
 */
#define PREFERRED_NODE_ID 1

// implement a 4-in-1 ESC
#define NUM_ESCS 1

/*
  create a CanardInterface class for interfacing with the hardware
 */
class CanardInterface : public Canard::Interface {
    friend class ESCNode;

    CanardInterface(uint8_t iface_index) :
        Interface(iface_index) {}
    
public:
    void init(const char *interface_name);

    // implement required interface functions
    bool broadcast(const Canard::Transfer &bcast_transfer) override;
    bool request(uint8_t destination_node_id, const Canard::Transfer &req_transfer) override;
    bool respond(uint8_t destination_node_id, const Canard::Transfer &res_transfer) override;

    void process(uint32_t duration_ms);

    static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer);
    static bool shouldAcceptTransfer(const CanardInstance* ins,
                                     uint64_t* out_data_type_signature,
                                     uint16_t data_type_id,
                                     CanardTransferType transfer_type,
                                     uint8_t source_node_id);

    uint8_t get_node_id() const override { return canard.node_id; }
    void set_node_id(uint8_t node_id) {
        canardSetLocalNodeID(&canard, node_id);
    }

private:
    uint8_t memory_pool[2048];
    CanardInstance canard;
    CanardTxTransfer tx_transfer;

    // we will use socketcan driver for this example
    SocketCANInstance socketcan;
};

/*
  declare heads of handler and transfer lists
 */
DEFINE_HANDLER_LIST_HEADS();
DEFINE_TRANSFER_OBJECT_HEADS();

class ESCNode {
public:
    void start_node(const char *interface_name);

private:
    CanardInterface canard_iface{0};

    // declare publishers for outgoing messages
    Canard::Publisher<uavcan_protocol_NodeStatus> node_status{canard_iface};
    Canard::Publisher<uavcan_equipment_esc_Status> esc_status{canard_iface};
    Canard::Publisher<uavcan_equipment_esc_RPMCommand> rpm_pub{canard_iface};

    void handle_GetNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoResponse& rsp);
    Canard::ObjCallback<ESCNode, uavcan_protocol_GetNodeInfoResponse> get_node_info_cb{this, &ESCNode::handle_GetNodeInfo};
    Canard::Client<uavcan_protocol_GetNodeInfoResponse> get_node_info_client{canard_iface, get_node_info_cb};

    void handle_ParamExecuteOpcode(const CanardRxTransfer& transfer, const uavcan_protocol_param_ExecuteOpcodeResponse& rsp);
    Canard::ObjCallback<ESCNode, uavcan_protocol_param_ExecuteOpcodeResponse> param_execute_cb{this, &ESCNode::handle_ParamExecuteOpcode};
    Canard::Client<uavcan_protocol_param_ExecuteOpcodeResponse> param_execute_client{canard_iface, param_execute_cb};


    void send_NodeStatus(void);
    void process1HzTasks(uint64_t timestamp_usec);
    void send_ESCStatus(void);

    /*
      keep the state of 4 ESCs, simulating a 4 in 1 ESC node
    */
    struct esc_state {
        float throttle;
        uint64_t last_update_us;
    } escs[NUM_ESCS];

    // keep node_status around for updating status
    uavcan_protocol_NodeStatus node_status_msg;

    /*
      data for dynamic node allocation process
    */
    struct {
        uint32_t send_next_node_id_allocation_request_at_ms;
        uint32_t node_id_allocation_unique_id_offset;
    } DNA;

    static struct parameter {
        const char *name;
        enum uavcan_protocol_param_Value_type_t type;
        float value;
        float min_value;
        float max_value;
    } parameters[];
};

/*
  a set of parameters to present to the user. In this example we don't
  actually save parameters, this is just to show how to handle the
  parameter protocool
 */
ESCNode::parameter ESCNode::parameters[] = {
    { "CAN_NODE", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, MY_NODE_ID, 0, 127 },
};

/*
  get a 64 bit monotonic timestamp in microseconds since start. This
  is platform specific
 */
static uint64_t micros64(void)
{
    static uint64_t first_us;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t tus = (uint64_t)(ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL);
    if (first_us == 0) {
        first_us = tus;
    }
    return tus - first_us;
}

/*
  get monotonic time in milliseconds since startup
 */
static uint32_t millis32(void)
{
    return micros64() / 1000ULL;
}

bool CanardInterface::broadcast(const Canard::Transfer &bcast_transfer) {
    tx_transfer = {
        .transfer_type = bcast_transfer.transfer_type,
        .data_type_signature = bcast_transfer.data_type_signature,
        .data_type_id = bcast_transfer.data_type_id,
        .inout_transfer_id = bcast_transfer.inout_transfer_id,
        .priority = bcast_transfer.priority,
        .payload = (const uint8_t*)bcast_transfer.payload,
        .payload_len = uint16_t(bcast_transfer.payload_len),
#if CANARD_ENABLE_CANFD
        .canfd = bcast_transfer.canfd,
#endif
#if CANARD_ENABLE_DEADLINE
        .deadline_usec = micros64() + (bcast_transfer.timeout_ms * 1000),
#endif
#if CANARD_MULTI_IFACE
        .iface_mask = uint8_t((1<<num_ifaces) - 1),
#endif
    };
    // do canard broadcast
    bool success = canardBroadcastObj(&canard, &tx_transfer) > 0;
    return success;
}

bool CanardInterface::request(uint8_t destination_node_id, const Canard::Transfer &req_transfer) {
    tx_transfer = {
        .transfer_type = req_transfer.transfer_type,
        .data_type_signature = req_transfer.data_type_signature,
        .data_type_id = req_transfer.data_type_id,
        .inout_transfer_id = req_transfer.inout_transfer_id,
        .priority = req_transfer.priority,
        .payload = (const uint8_t*)req_transfer.payload,
        .payload_len = uint16_t(req_transfer.payload_len),
#if CANARD_ENABLE_CANFD
        .canfd = req_transfer.canfd,
#endif
#if CANARD_ENABLE_DEADLINE
        .deadline_usec = micros64() + (req_transfer.timeout_ms * 1000),
#endif
#if CANARD_MULTI_IFACE
        .iface_mask = uint8_t((1<<num_ifaces) - 1),
#endif
    };
    // do canard request
    return canardRequestOrRespondObj(&canard, destination_node_id, &tx_transfer) > 0;
}

bool CanardInterface::respond(uint8_t destination_node_id, const Canard::Transfer &res_transfer) {
    tx_transfer = {
        .transfer_type = res_transfer.transfer_type,
        .data_type_signature = res_transfer.data_type_signature,
        .data_type_id = res_transfer.data_type_id,
        .inout_transfer_id = res_transfer.inout_transfer_id,
        .priority = res_transfer.priority,
        .payload = (const uint8_t*)res_transfer.payload,
        .payload_len = uint16_t(res_transfer.payload_len),
#if CANARD_ENABLE_CANFD
        .canfd = res_transfer.canfd,
#endif
#if CANARD_ENABLE_DEADLINE
        .deadline_usec = micros64() + (res_transfer.timeout_ms * 1000),
#endif
#if CANARD_MULTI_IFACE
        .iface_mask = uint8_t((1<<num_ifaces) - 1),
#endif
    };
    // do canard respond
    return canardRequestOrRespondObj(&canard, destination_node_id, &tx_transfer) > 0;
}

void ESCNode::handle_GetNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoResponse& rsp)
{
    // (void)req;
    // uavcan_protocol_GetNodeInfoResponse rsp;
    printf("Got GetNodeInfo response\n");
    printf("ESC name: ");
    for(int i = 0; i < rsp.name.len; i++) {
        printf("%c", rsp.name.data[i]);
    }
    printf("\n");

    printf("Hardware version: %u\n", rsp.hardware_version.major);
    printf("Software version: %u\n", rsp.software_version.major);
    printf("Software VCS: %u\n", rsp.software_version.vcs_commit);
    printf("Status: %u\n", rsp.status.health);

    uavcan_equipment_esc_RPMCommand rpm_cmd;

    int32_t rpm = 10;

    rpm_cmd.rpm.data[0] = rpm;
    rpm_cmd.rpm.len = 1;

    bool is_broadcasted = rpm_pub.broadcast(rpm_cmd);

    printf("Broadcasted RPM: %d \t", rpm);
    printf("Broadcasted: %d\n", is_broadcasted);

}

void ESCNode::handle_ParamExecuteOpcode(const CanardRxTransfer& transfer, const uavcan_protocol_param_ExecuteOpcodeResponse& rsp)
{
    // (void)rsp;
    // uavcan_protocol_param_ExecuteOpcodeResponse rsp {};

}

/*
  send the 1Hz NodeStatus message. This is what allows a node to show
  up in the DroneCAN GUI tool and in the flight controller logs
 */
void ESCNode::send_NodeStatus(void)
{
    node_status_msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status_msg.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status_msg.sub_mode = 0;
    node_status_msg.uptime_sec = millis32() / 1000UL;

    node_status.broadcast(node_status_msg);
}

/*
  This function is called at 1 Hz rate from the main loop.
*/
void ESCNode::process1HzTasks(uint64_t timestamp_usec)
{
    /*
      Transmit the node status message
    */
    send_NodeStatus();
}

/*
  send ESC status at 50Hz
*/
void ESCNode::send_ESCStatus(void)
{
    // send a separate status packet for each ESC
    for (uint8_t i=0; i<NUM_ESCS; i++) {
        uavcan_equipment_esc_Status pkt {};

        // make up some synthetic status data
        pkt.error_count = 0;
        pkt.voltage = 16.8 - 2.0 * escs[i].throttle;
        pkt.current = 20 * escs[i].throttle;
        pkt.temperature = 298.0;
        pkt.rpm = 10000 * escs[i].throttle;
        pkt.power_rating_pct = 100.0 * escs[i].throttle;

        esc_status.broadcast(pkt);
    }
}


/*
  Transmits all frames from the TX queue, receives up to one frame.
*/
void CanardInterface::process(uint32_t timeout_msec)
{
    // Transmitting
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
        const int16_t tx_res = socketcanTransmit(&socketcan, txf, 0);
        if (tx_res != 0) {
            canardPopTxQueue(&canard);
        }
        else {
            break;
        }
    }

    // Receiving
    CanardCANFrame rx_frame;

    const uint64_t timestamp = micros64();
    const int16_t rx_res = socketcanReceive(&socketcan, &rx_frame, timeout_msec);
    if (rx_res > 0) {
        canardHandleRxFrame(&canard, &rx_frame, timestamp);
    }
    else if(rx_res < 0) {
        (void)fprintf(stderr, "Receive error %d, errno '%s'\n", rx_res, strerror(errno));
    }
}

/*
  handle an incoming message
 */
void CanardInterface::onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
    CanardInterface* iface = (CanardInterface*) ins->user_reference;
    iface->handle_message(*transfer);
}

/*
  check if we want the message. This is based on what we have subscribed to
 */
bool CanardInterface::shouldAcceptTransfer(const CanardInstance* ins,
                                   uint64_t* out_data_type_signature,
                                   uint16_t data_type_id,
                                   CanardTransferType transfer_type,
                                   uint8_t source_node_id)
{
    CanardInterface* iface = (CanardInterface*)ins->user_reference;
    return iface->accept_message(data_type_id, *out_data_type_signature);
}

/*
  Initializing the Libcanard instance.
*/
void CanardInterface::init(const char *interface_name)
{
    int16_t res = socketcanInit(&socketcan, interface_name);
    if (res < 0) {
        (void)fprintf(stderr, "Failed to open CAN iface '%s'\n", interface_name);
        exit(1);
    }

    // init canard object
    canardInit(&canard,
               memory_pool,
               sizeof(memory_pool),
               onTransferReceived,
               shouldAcceptTransfer,
               this);

    // set node ID if not doing DNA
    if (MY_NODE_ID > 0) {
        canardSetLocalNodeID(&canard, MY_NODE_ID);
    } else {
        printf("Waiting for DNA node ID allocation\n");
    }
}

/*
 Initializing the CAN backend driver; in this example we're using SocketCAN
 */
void ESCNode::start_node(const char *interface_name)
{
    // init the interface
    canard_iface.init(interface_name);

    uint8_t node_id = canard_iface.get_node_id();

    printf("ESCNode started on %s, node ID %d\n", interface_name, node_id);

    send_NodeStatus();

    canard_iface.process(100);

    /*
      Run the main loop.
     */
    uint64_t next_1hz_service_at = micros64();
    uint64_t next_50hz_service_at = micros64();

    int32_t rpm = 10;

    uavcan_equipment_esc_RPMCommand rpm_cmd;

    rpm_cmd.rpm.data[0] = rpm;
    rpm_cmd.rpm.len = 4;

    uavcan_protocol_GetNodeInfoRequest req;

    req = {};

    while(get_node_info_client.request(1,req) == false) {
        printf("Requesting Node Info\n");
        canard_iface.process(10);
    }
    
    rpm = 2000;

    rpm_cmd.rpm.data[0] = rpm;
    rpm_cmd.rpm.len = 4;

    while (true) {

        uint64_t ts = micros64();


        if (ts >= next_1hz_service_at) {
            send_NodeStatus();
            next_1hz_service_at += 1000000ULL;
        //     process1HzTasks(ts);
        }
        if (ts >= next_50hz_service_at) {
            next_50hz_service_at += 1000000ULL/100U;
            rpm_pub.broadcast(rpm_cmd);
        //     // printf("Broadcasting RPM: %d\n", rpm);
        }
        canard_iface.process(10);
    }
}

// declare our ESC node
static ESCNode node;

/*
  main program
 */
int main(int argc, char** argv)
{
    if (argc < 2) {
        (void)fprintf(stderr,
                      "Usage:\n"
                      "\t%s <can iface name>\n",
                      argv[0]);
        return 1;
    }

    node.start_node(argv[1]);
    return 0;
}
