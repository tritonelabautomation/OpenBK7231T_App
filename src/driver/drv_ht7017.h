#pragma once

#ifndef __DRV_HT7017_H__
#define __DRV_HT7017_H__

// Only expose the public functions
void HT7017_Init(void);
void HT7017_RunEverySecond(void);
void HT7017_AppendInformationToHTTPIndexPage(http_request_t* request);

#endif
