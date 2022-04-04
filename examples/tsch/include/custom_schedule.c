#include "custom_schedule.h"
#include "net/ipv6/uip-ds6-route.h"
#include "net/ipv6/uip-sr.h"
#ifdef MAC_CONF_WITH_TSCH
#include "net/mac/tsch/tsch.h"


static linkaddr_t node_1_address = {
    {0x00, 0x12, 0x4b, 0x00, 0x19, 0x32, 0xe6, 0x92}};
static linkaddr_t node_2_address = {
    {0x00, 0x12, 0x4b, 0x00, 0x18, 0xe6, 0x9c, 0x9e}};
static linkaddr_t node_3_address = {
    {0x00, 0x12, 0x4b, 0x00, 0x14, 0xb5, 0x2d, 0xbc}};

void tsch_custom_schedule_init(void) {
  struct tsch_slotframe *sf_custom;
  /* First, empty current schedule */
  tsch_schedule_remove_all_slotframes();
  /* Build 6TiSCH minimal schedule.
   * We pick a slotframe length of TSCH_SCHEDULE_DEFAULT_LENGTH */
  sf_custom = tsch_schedule_add_slotframe(0, TSCH_SCHEDULE_DEFAULT_LENGTH);
  /* Add a single Tx|Rx|Shared slot using broadcast address (i.e. usable for
   * unicast and broadcast). We set the link type to advertising, which is not
   * compliant with 6TiSCH minimal schedule but is required according to
   * 802.15.4e if also used for EB transmission.
   * Timeslot: 0, channel offset: 0. */
  tsch_schedule_add_link(sf_custom,
                         LINK_OPTION_RX | LINK_OPTION_TX | LINK_OPTION_SHARED |
                             LINK_OPTION_TIME_KEEPING,
                         LINK_TYPE_ADVERTISING, &tsch_broadcast_address, 0, 0,
                         0);

  if (linkaddr_node_addr.u8[7] == node_1_address.u8[7]) {
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL,
                           &node_2_address, 1, 0, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL,
                           &node_2_address, 2, 0, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL,
                           &node_3_address, 3, 0, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL,
                           &node_3_address, 4, 0, 0);
  } else if (linkaddr_node_addr.u8[7] == node_2_address.u8[7]) {
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL,
                           &node_1_address, 1, 0, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL,
                           &node_1_address, 2, 0, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL,
                           &node_3_address, 5, 0, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL,
                           &node_3_address, 6, 0, 0);
  } else if (linkaddr_node_addr.u8[7] == node_3_address.u8[7]) {
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL,
                           &node_1_address, 3, 0, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL,
                           &node_1_address, 4, 0, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_TX, LINK_TYPE_NORMAL,
                           &node_2_address, 5, 0, 0);
    tsch_schedule_add_link(sf_custom, LINK_OPTION_RX, LINK_TYPE_NORMAL,
                           &node_2_address, 6, 0, 0);
  }
}
#else
void tsch_custom_schedule_init(void);
#endif
