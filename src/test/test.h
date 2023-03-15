#ifndef _TEST_H_
#define _TEST_H_


#if defined(TEST_READING_DEV_ID)
void read_dev_id(void *p1, void *p2, void *p3);
#endif

#if defined(TEST_AES_SS_TWR_INITIATOR)
void ss_aes_twr_initiator(void *p1, void *p2, void *p3);
#endif

#if defined(TEST_AES_SS_TWR_RESPONDER)
void ss_aes_twr_responder(void *p1, void *p2, void *p3);
#endif

#endif