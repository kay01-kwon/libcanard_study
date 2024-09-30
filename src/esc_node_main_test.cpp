#include "canard_interface/canard_interface.hpp"
#include "canard_interface/drone_can_node.hpp"

int main(int argc, char** argv)
{
    if (argc < 2) {
        (void)fprintf(stderr,
                      "Usage:\n"
                      "\t%s <can iface name>\n",
                      argv[0]);
        return 1;
    }

    DroneCanNode node;
    node.start_node(argv[1]);
    return 0;
}