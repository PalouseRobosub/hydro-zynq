#include "network_stack.h"

#include "netif/xadapter.h"

/**
 * The ethernet networking interface used for communication.
 */
static struct netif ethernet_interface;

result_t init_network_stack(struct ip_addr ip_address, struct ip_addr netmask,
                            struct ip_addr gateway, macaddr_t mac_address)
{
    /*
     * Initialize the lwIP network stack.
     */
    lwip_init();

    /*
     * Add the ethernet interface to the lwIP network stack using Xilinx's
     * wrapper function. Note that this function handles Xilinx-specific
     * initialization of the EMAC driver. Data will be handed to lwIP from
     * an interrupt context. Xilinx's initialize code registers the interrupt
     * and handles all data flow into and out of lwIP's network stack.
     */
    AbortIfNot(xemac_add(&ethernet_interface,
                         &ip_address,
                         &netmask,
                         &gateway,
                         mac_address.addr,
                         PLATFORM_EMAC_BASE_ADDRESS), fail);

    netif_set_default(&ethernet_interface);

    /*
     * Specify that the ethernet interface is up.
     */
    netif_set_up(&ethernet_interface);

    /*
     * Set up the TCP fast and slow timer interrupts.
     */
    //TODO: Determine if lwIP handles TCP timers internally.

    return success;
}
