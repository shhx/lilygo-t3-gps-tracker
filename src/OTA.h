#ifndef OTA_H
#define OTA_H

void setupWiFiOTA(const char* ssid, const char* password); 
void setupOTA(const char* nameprefix);
void startOTATask();

#endif
