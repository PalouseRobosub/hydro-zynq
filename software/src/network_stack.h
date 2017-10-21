#ifndef NETWORK_STACK_H
#define NETWORK_STACK_H

#include "types.h"
#include "lwip/ipaddr.h"

typedef struct macaddr_t
{
    uint8_t addr[6];
} macaddr_t;

result_t init_network_stack(struct ip_addr ip_address, struct ip_addr netmask,
                            struct ip_addr gateway, macaddr_t mac_address);


#endif
