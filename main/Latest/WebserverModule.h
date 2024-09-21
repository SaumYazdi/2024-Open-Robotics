#ifndef WebserverModule_h
#define WebserverModule_h

#include <WiFi.h>
#include <WebServer.h>

class WebserverModule {
  public:
    WebserverModule();
    void update();
    // Webserver routes
    void handle_OnConnect();
    void handle_speed();
    void handle_direction();
    void handle_NotFound();
  private:
    int speed = 0;
    int direction = 0;
    String status = "Connected";
    // Specifying the SSID and Password of the AP
    const char* ap_ssid = "Shenzhen Weinan Electronics Co."; // Access Point SSID
    const char* ap_password= "admin654"; // Access Point Password
    uint8_t max_connections=1; // Maximum Connection Limit for AP
    int current_stations=0, new_stations=0;
    IPAddress local_IP(192, 168, 4, 1);  // Set your desired static IP address
    IPAddress gateway(192, 168, 4, 1);   // Usually the same as the IP address
    IPAddress subnet(255, 255, 255, 0);
    IPAddress IP;
    // Specifying the Webserver instance to connect with HTTP Port: 80
    WebServer server(80);
    // Global robot values
    String HTML();
};

#endif