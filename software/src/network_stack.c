#include "network_stack.h"

#include "netif/xadapter.h"
#include "xparameters.h"
#include "abort.h"
#include "db.h"
#include "lwip/init.h"

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
                         XPAR_XEMACPS_0_BASEADDR), fail);

    netif_set_default(&ethernet_interface);

    /*
     * Specify that the ethernet interface is up.
     */
    netif_set_up(&ethernet_interface);

    return success;
}

/**
 * Forward traffic received rom the Ethernet driver into the network stack.
 *
 * @return None.
 */
void dispatch_network_stack()
{
    uint32_t packets_rx = xemacif_input(&ethernet_interface);
    if (packets_rx)
    {
        dbprintf("Received %d packets\n", packets_rx);
    }
}
