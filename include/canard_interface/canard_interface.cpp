#include "canard_interface.hpp"

void CanardInterface::init(const char *interface_name)
{
    int16_t result = socketcanInit(&socketcan_, interface_name);
    if(result < 0)
    {
        std::cerr << "Failed to initialize the socketcan interface" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Initialize canard object
    canardInit( &canard_, 
                memory_pool_, 
                sizeof(memory_pool_), 
                onTransferReceived, 
                shouldAcceptTransfer, 
                this);

    // Set the node id
    canardSetLocalNodeID(&canard_, 127);
}

bool CanardInterface::broadcast(const Canard::Transfer &transfer)
{
    tx_transfer_ = {
        .transfer_type = transfer.transfer_type,
        .data_type_signature = transfer.data_type_signature,
        .data_type_id = transfer.data_type_id,
        .inout_transfer_id = transfer.inout_transfer_id,
        .priority = transfer.priority,
        .payload = (const uint8_t *)transfer.payload,
        .payload_len = uint16_t(transfer.payload_len),
    };

    return canardBroadcastObj(&canard_, &tx_transfer_) > 0;
}

bool CanardInterface::request(uint8_t dest_node_id, 
const Canard::Transfer &transfer)
{
    tx_transfer_ = {
        .transfer_type = transfer.transfer_type,
        .data_type_signature = transfer.data_type_signature,
        .data_type_id = transfer.data_type_id,
        .inout_transfer_id = transfer.inout_transfer_id,
        .priority = transfer.priority,
        .payload = (const uint8_t *)transfer.payload,
        .payload_len = uint16_t(transfer.payload_len),
    };

    return canardRequestOrRespondObj;
}

bool CanardInterface::respond(uint8_t dest_node_id, const Canard::Transfer &transfer)
{
    tx_transfer_ = {
        .transfer_type = transfer.transfer_type,
        .data_type_signature = transfer.data_type_signature,
        .data_type_id = transfer.data_type_id,
        .inout_transfer_id = transfer.inout_transfer_id,
        .priority = transfer.priority,
        .payload = (const uint8_t *)transfer.payload,
        .payload_len = uint16_t(transfer.payload_len),
    };
    return canardRequestOrRespondObj(&canard_, dest_node_id, &tx_transfer_) > 0;
}

void CanardInterface::process(uint32_t duration_ms)
{
}

void CanardInterface::onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
}

bool CanardInterface::shouldAcceptTransfer(const CanardInstance *ins, 
uint64_t *out_data_type_signature, 
uint16_t data_type_id, 
CanardTransferType transfer_type, 
uint8_t source_node_id)
{
    return false;
}
