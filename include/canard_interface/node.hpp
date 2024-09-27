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