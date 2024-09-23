#ifndef WebserverModule_h
#define WebserverModule_h

#include <WiFi.h>
#include <WebServer.h>

class WebserverModule {
  public:
    WebserverModule();
    void setup();
    void update();

    // Webserver routes
    void handle_OnConnect();
    void handle_speed();
    void handle_direction();
    void handle_NotFound();

  private:
    int speed;
    int direction;
    String status;

    // Specifying the SSID and Password of the AP
    const char* ap_ssid = "Shenzhen Weinan Electronics Co."; // Access Point SSID
    const char* ap_password = "admin654"; // Access Point Password
    uint8_t max_connections = 1; // Maximum Connection Limit for AP
    int current_stations = 0, new_stations = 0;

    // Specifying the Webserver instance to connect with HTTP Port: 80
    WebServer* server;

    // Global robot values
    String HTML();
};

#endif