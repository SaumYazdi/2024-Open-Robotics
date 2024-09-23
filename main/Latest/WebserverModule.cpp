#include "WebserverModule.h"

IPAddress local_IP(192, 168, 4, 1);  // Set your desired static IP address
IPAddress gateway(192, 168, 4, 1);   // Usually the same as the IP address
IPAddress subnet(255, 255, 255, 0);
IPAddress IP;

WebserverModule::WebserverModule() {
  speed = 0;
  direction = 0;
  status = "Connected";

  server = new WebServer(80);
  
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_IP, gateway, subnet); // Configure static IP
  
  //Setting the AP Mode with SSID, Password, and Max Connection Limit
  if(WiFi.softAP(ap_ssid,ap_password,1,false,max_connections)==true) {
    Serial.print("Access Point is Created with SSID: ");
    Serial.println(ap_ssid);
    Serial.print("Max Connections Allowed: ");
    Serial.println(max_connections);
    Serial.print("Access Point IP: ");
    Serial.println(WiFi.softAPIP());
  }
  else {
    Serial.println("Unable to Create Access Point");
  }
}

void WebserverModule::setup() {  
  // Start the serial communication channel
  Serial.begin(9600);
  while (!Serial); // Wait until serial is available
  Serial.println();

  //Specifying the functions which will be executed upon corresponding GET request from the client
  server->on("/", HTTP_GET, handle_OnConnect);
  server->on("/speed", handle_speed);
  server->on("/direction", handle_direction);
  server->onNotFound(handle_NotFound);
   
  //Starting the Server
  server->begin();
  Serial.println("HTTP Server Started");
}
 
void WebserverModule::update() {
  // Assign the server to handle the clients
  server->handleClient();
     
  //Continuously check how many stations are connected to Soft AP and notify whenever a new station is connected or disconnected
  new_stations = WiFi.softAPgetStationNum();
   
  if(current_stations < new_stations)//Device is Connected
  {
    current_stations = new_stations;
    Serial.print("New Device Connected to SoftAP... Total Connections: ");
    Serial.println(current_stations);
  }
   
  if(current_stations > new_stations)//Device is Disconnected
  {
    current_stations = new_stations;
    Serial.print("Device disconnected from SoftAP... Total Connections: ");
    Serial.println(current_stations);
  }
}
 
void WebserverModule::handle_OnConnect() {
  status = "Connected to AP";
  Serial.println("Client Connected");
  server->send(200, "text/html", HTML()); 
}

void WebserverModule::handle_speed() {
  speed = server->arg("value").toInt();
  Serial.println(speed);
  server->send(200, "text/html", HTML());
}

void WebserverModule::handle_direction() {
  direction = server->arg("value").toInt();
  Serial.println(direction);
  server->send(200, "text/html", HTML());
}

void WebserverModule::handle_NotFound() {
  server->send(404, "text/plain", "Not found");
}
 
String WebserverModule::HTML() {
  const char index_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE html>
    <html>
      <head>
        <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
        <title>Robot Controller</title>
        <style>
          html {font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}
          body {margin-top: 50px;} h1 {color: #444444; margin: 50px auto 30px;} h3 {color: #444444; margin-bottom: 50px;}
          .button {display: block; width: 180px; background-color: #0d81ec; border: none; color: white; padding: 13px 30px; text-decoration: none; font-size: 25px; margin: 0px auto 35px; cursor: pointer; border-radius: 4px;}
          .button-update {background-color: #0d81ec;}
          .button-update:active {background-color: #0d81ec;}
          .textbox {width: 200px; height: 30px; border: none; background-color: #f48100; color: white; padding: 5px; font-size: 16px; border-radius: 4px; margin: 0px auto 35px;}
          input[type="range"] {-webkit-appearance: none; appearance: none; background: transparent; cursor: pointer; width: 15rem;}
          input[type="range"]:focus {outline: none;}
          input[type="range"]::-webkit-slider-runnable-track {background-color: #afafaf; border-radius: 0.5rem; height: 0.5rem;}
          input[type="range"]::-webkit-slider-thumb {-webkit-appearance: none; appearance: none; margin-top: -12px; background-color: #5cd5eb; height: 2rem; width: 1rem; border-radius: 0.5rem;}
          input[type="range"]:focus::-webkit-slider-thumb {border: 1px solid #afafaf; outline: 3px solid #afafaf; outline-offset: 0.125rem;}
          .status-field {width: 300px; height: 30px; border: none; background-color: #ffffff; color: black; padding: 5px; font-size: 16px; border-radius: 4px; margin: 0px auto 15px;}
        </style>
      </head>

      <body>
        <h1>Robot Controller</h1>
        <h3>Using Access Point (AP) Mode</h3>
        
        <input type='range' value=0 min=0 max=100 id="speedSlider">
        <p id="speedLabel">Speed: 0</p>
        <script type="text/javascript">
          var speedSlider = document.getElementById("speedSlider");
          var speedLabel = document.getElementById("speedLabel");
          function updateSpeed() {
            var speed = speedSlider.value;
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/slider?value=" + speed, true);
            xhr.send();
            speedLabel.innerHTML = "Speed: " + speed;
          }
          speedSlider.addEventListener("change", updateSpeed);
        </script>
        
        <input type='range' value=0 min=0 max=360 id="directionSlider">
        <p id="directionLabel">Direction: 0</p>
        <script type="text/javascript">
          var directionSlider = document.getElementById("directionSlider");
          var directionLabel = document.getElementById("directionLabel");
          function updateDirection() {
            var direction = directionSlider.value;
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/direction?value=" + direction, true);
            xhr.send();
            directionLabel.innerHTML = "Direction: " + direction;
          }
          directionSlider.addEventListener("change", updateDirection);
        </script>
        
      </body>
    </html>
  )rawliteral";

  return index_html;
}