#ifndef CANARD_INTERFACE_HPP
#define CANARD_INTERFACE_HPP

#include "drone_can_node.hpp"

class CanardInterface : public Canard::Interface{


    public:

        friend class DroneCanNode;

        CanardInterface(uint8_t iface_index)
        : Canard::Interface(iface_index)
        {}

    // Implement the Canard::Interface pure virtual functions
    void init(const char *interface_name);

    bool broadcast(const Canard::Transfer &transfer) override;
    
    bool request(uint8_t dest_node_id,
                const Canard::Transfer &transfer) override;

    bool respond(uint8_t dest_node_id,
                const Canard::Transfer &transfer) override;

    uint8_t get_node_id() const override
    {
        return canard_.node_id;
    }

    void process(uint32_t duration_ms);

    static void onTransferReceived(CanardInstance* ins,
                                CanardRxTransfer* transfer);
    
    static bool shouldAcceptTransfer(const CanardInstance* ins,
                                    uint64_t* out_data_type_signature,
                                    uint16_t data_type_id,
                                    CanardTransferType transfer_type,
                                    uint8_t source_node_id);
    

    private:

        uint8_t memory_pool_[2048];
        CanardInstance canard_;
        CanardTxTransfer tx_transfer_;

        SocketCANInstance socketcan_;

};

#endif // CANARD_INTERFACE_HPP