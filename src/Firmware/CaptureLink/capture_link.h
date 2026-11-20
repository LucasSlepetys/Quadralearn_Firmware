#ifndef CAPTURE_LINK_H
#define CAPTURE_LINK_H

#include "EndpointCapture/endpoint_capture.h"

typedef struct {
    char line[96];
    int length;
    int overflow;
    int active_leg;
    unsigned long preview_token; //Identifies the exact preview shown in Python.
    int (*other_request)(const char *line, void *context); //Optional bench routing.
    void *other_context;
} CaptureLink;

//USB data interface only; no terminal menus or PWM.
void capture_link_initialize(CaptureLink *link);
void capture_link_update(CaptureLink *link, EndpointCaptureState *captures);

#endif // CAPTURE_LINK_H
