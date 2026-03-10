/*
 * drv_kws303wf.h — Public API for KWS-303WF Device Application Driver
 *
 * Only two functions are public — the OBK lifecycle functions.
 * Everything else (relay, buttons, NTC, session) is internal to drv_kws303wf.c
 */

#ifndef DRV_KWS303WF_H
#define DRV_KWS303WF_H

#include "../obk_config.h"
#if ENABLE_DRIVER_KWS303WF

void KWS303WF_Init(void);
void KWS303WF_RunEverySecond(void);
void KWS303WF_RunQuickTick(void);
void KWS303WF_OnHassDiscovery(const char *topic);

#endif /* ENABLE_DRIVER_KWS303WF */
#endif /* DRV_KWS303WF_H */
