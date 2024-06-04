#include <stdio.h>
#include <inttypes.h>

#include "uorb_fmu.h"

/*
 * Utility to listen on uORB topics and print the data to the console.                                                                                     
 *                                                                                                                                                         
 * The listener can be exited any time by pressing Ctrl+C, Esc, or Q.                                                                                      
 *                                                                                                                                                         
 * We use cmd 'uorb_listener <t1,t2,...> -n 1' to print once snapshot for                                                                                  
 * specify topics if command exists or all the topic objects when command                                                                                  
 * doesn't exists. If topic isn't updated, the snapshot for this topic is                                                                                  
 * empty.                                                                                                                                                  
 *                                                                                                                                                         
 * We use cmd 'uorb_listener t1,t2,... [arguments...]' to print the topics                                                                                 
 * specified. The following arguments apply to all topics.                                                                                                 
 *                                                                                                                                                         
 * We use cmd 'uorb_listener ... -n num(>1) ...' to print all the topic                                                                                    
 * objects message until the number of received is equal to the specified                                                                                  
 * number.                                                                                                                                                 
 *                                                                                                                                                         
 * We can print the messages all the time when we do not specify the number                                                                                
 * of message.                                                                                                                                             
 *                                                                                                                                                         
 * listener <command> [arguments...]                                                                                                                       
 *  Commands:                                                                                                                                              
 *         <topics_name> Topic name. Multi name are separated by ','                                                                                       
 *         [-h       ]  Listener commands help                                                                                                             
 *         [-n <val> ]  Number of messages, default: 0                                                                                                     
 *         [-r <val> ]  Subscription rate (unlimited if 0), default: 0                                                                                     
 *         [-b <val> ]  Subscription maximum report latency in us(unlimited if 0),                                                                         
 *                      default: 0                                                                                                                         
 *         [-t <val> ]  Time of listener, in seconds, default: 5                                                                                           
 *         [-T       ]  Top, continuously print updating objects                                                                                           
 *         [-l       ]  Top only execute once.
 */

#ifdef CONFIG_DEBUG_UORB
static void print_orb_gps_raw_msg(FAR const struct orb_metadata *meta, FAR const void *buffer) {
    FAR const struct orb_gps_raw_gga_t *message = (orb_gps_raw_gga_t *)buffer;
    const orb_abstime now = orb_absolute_time();

    printf("%s: ttimestamp: %" PRIu64 " (%" PRIu64 " us ago)\n%s\n",
        meta->o_name,
        message->timestamp,
        now - message->timestamp,
        message->gga_raw);
}

static void print_orb_rtcm_msg(FAR const struct orb_metadata *meta, FAR const void *buffer) {
    FAR const struct orb_rtcm_t *message = (orb_rtcm_t *)buffer;
    const orb_abstime now = orb_absolute_time();

    printf("%s: ttimestamp: %" PRIu64 " (%" PRIu64 " us ago)\nRTCM len: %" PRIu16 "\nRTCM  id: %" PRIu16 "\n",
        meta->o_name,
        message->timestamp,
        now - message->timestamp,
        message->rtcm_len,
        message->rtcm_id);
}

#endif

ORB_DEFINE(orb_gps_gga_raw,     struct orb_gps_raw_gga_t,   print_orb_gps_raw_msg);
ORB_DEFINE(orb_rtcm_unicore,    struct orb_rtcm_t,          print_orb_rtcm_msg);
ORB_DEFINE(orb_rtcm_ntrip,      struct orb_rtcm_t,          print_orb_rtcm_msg);
