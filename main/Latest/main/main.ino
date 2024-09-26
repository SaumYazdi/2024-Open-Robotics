#include "Bot.h"
#include <WiFi.h>
#include <WebServer.h>

Bot* bot;

// Global robot values
int speed = 0;
int direction = 0;

const float tofAngles[8] = { -33.5, -68.5, -113.5, -158.5, 158.5, 113.5, 68.5, 33.5 };

// Webserver routes
void handle_OnConnect();
void handle_speed();
void handle_direction();
void handle_NotFound();

String HTML();
String status = "Connected";

// Specifying the SSID and Password of the AP
const char* ap_ssid = "Shenzhen Weinan Electronics Co."; // Access Point SSID
const char* ap_password= "admin654"; // Access Point Password
uint8_t max_connections = 1; // Maximum Connection Limit for AP
int current_stations = 0, new_stations = 0;

IPAddress local_IP(192, 168, 4, 1);  // Set your desired static IP address
IPAddress gateway(192, 168, 4, 1);   // Usually the same as the IP address
IPAddress subnet(255, 255, 255, 0);
IPAddress IP;

// Specifying the Webserver instance to connect with HTTP Port: 80
WebServer server(80);

void setup() {
  // Pi5 Serial
  Serial.end();
  Serial.begin(115200);
  bot = new Bot();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_IP, gateway, subnet); // Configure static IP
   
  // Setting the AP Mode with SSID, Password, and Max Connection Limit
  WiFi.softAP(ap_ssid,ap_password,1,false,max_connections);
 
  // Specifying the functions which will be executed upon corresponding GET request from the client
  server.on("/", HTTP_GET, handle_OnConnect);
  server.on("/speed", handle_speed);
  server.on("/direction", handle_direction);
  server.on("/update", handle_update);
  server.on("/dynamic_values", dynamic_values);
  server.onNotFound(handle_NotFound);

  // Starting the Server
  server.begin();
}

void loop() {
  bot->update();
  
  // Assign the server to handle the clients
  server.handleClient();
}
 
void handle_OnConnect() {
  status = "Connected to AP";
  server.send(200, "text/html", HTML()); 
}

void handle_update() {
  speed = server.arg("speed").toInt();
  direction = server.arg("direction").toInt();
  server.send(200, "text/html", HTML());

  bot->speed = speed;
  bot->direction = direction;
}

void handle_speed() {
  speed = server.arg("value").toInt();
  server.send(200, "text/html", HTML());

  bot->speed = speed;
}

void handle_direction() {
  direction = server.arg("value").toInt();
  server.send(200, "text/html", HTML());

  bot->direction = direction;
}

void handle_NotFound() {
  server.send(404, "text/plain", "Not found");
}

void dynamic_values() {
  String heading(bot->heading());

  int* tofs = bot->tofs();
  String distances = "{";
  for (int i = 0; i < 8; i++) {
    distances += "\"" + String(tofAngles[i]) + "\": " + String(tofs[i]);
    if (i < 7) {
      distances += ", ";
    }
  }
  distances += "}";

  String mode = bot->getMode();

  String json_data = "{\"heading\": " + heading + ", \"tofs\": " + distances + ", \"mode\": \"" + mode + "\"" + "}";

  String page = R"rawliteral(
    <head>
      <meta http-equiv=refresh content=0>
    </head>
  )rawliteral" + json_data +
  R"rawliteral(
  )rawliteral";

  server.send(200, "text/html", page);
}

String HTML() {
  const char index_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE html>
    <html>
      <head>
        <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
        <title>Robot Controller</title>
        <style>
          html {font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center; overscroll-behavior: none;}
          body {margin-top: 50px; overscroll-behavior: none;}
          h1 {color: #444444; margin: 50px auto 30px;}
          h3 {color: #444444; margin-bottom: 50px;}
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
        
        <button onclick="location.reload()">Refresh</button>

        <input type='range' value=0 min=0 max=90000000 id="speedSlider">
        <p id="speedLabel">Speed: 0</p>
        <script type="text/javascript">
          var speedSlider = document.getElementById("speedSlider");
          var speedLabel = document.getElementById("speedLabel");
          function updateSpeed() {
            var speed = speedSlider.value;
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/speed?value=" + speed, true);
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
        
        <canvas id="joystick" width="300" height="300"></canvas>
        <script type="text/javascript">
            var canvas = document.getElementById("joystick");
            let rect = canvas.getBoundingClientRect();
            let w = rect.width;
            let h = rect.height;
            let x = w / 2;
            let y = h / 2;
            let radius = 100;
            let innerRadius = 30;

            var previewContext = canvas.getContext("2d");
            
            function drawJoystick() {
                previewContext.fillStyle = 'rgb(255, 255, 255)';
                previewContext.fillRect(0, 0, 500, 500);

                previewContext.beginPath();
                previewContext.arc(w / 2, h / 2, radius, 0, 2 * Math.PI, false);
                previewContext.lineWidth = 5;
                previewContext.strokeStyle = 'rgb(50, 50, 50)';
                previewContext.stroke();

                previewContext.beginPath();
                previewContext.arc(x, y, innerRadius, 0, 2 * Math.PI, false);
                previewContext.fillStyle = 'rgb(120, 120, 120)';
                previewContext.fill();
                previewContext.strokeStyle = 'rgb(50, 50, 50)';
                previewContext.stroke();
            }
            drawJoystick();

            var xhr = new XMLHttpRequest();
            xhr.addEventListener("load", (event) => {console.log(event); canSend = true;});
            let canSend = true;
            window.blockMenuHeaderScroll = false;
            let ticks = 0;

            let prevSpeed;
            let prevDirection;
            window.onmousemove = (event) => {
                if (event.buttons != 1 && event.touches == undefined)
                    return;
                if (event.touches)
                    event = event.touches[0];

                let rect = canvas.getBoundingClientRect();
                x = (event.clientX - rect.left);
                y = (event.clientY - rect.top);

                dx = w / 2 - x;
                dy = h / 2 - y;
                let maxSpeed = 90000000;

                let dist = Math.sqrt(dx*dx + dy*dy);
                let direction = Math.atan2(dy, dx);

                if (dist > radius) {
                    if (ticks == 0)
                        return;
                    x = w / 2 - Math.cos(direction) * radius;
                    y = h / 2 - Math.sin(direction) * radius;
                }
                direction = direction * 180 / Math.PI - 90;
                let speed = Math.min(maxSpeed, dist * maxSpeed / radius);

                if (ticks % 1 == 0) {
                  speed = Math.round(speed);
                  direction = Math.round(direction) % 360;

                  speedLabel.innerHTML = "Speed: " + speed;
                  speedSlider.value = speed;
                  directionLabel.innerHTML = "Direction: " + direction;
                  directionSlider.value = direction;

                  if (canSend) {
                    if (prevSpeed != speed && prevDirection != direction) {
                      xhr.open("GET", `/update?speed=${speed}&direction=${direction}`, true);
                      xhr.send();
                      canSend = false;
                      prevSpeed = speed;
                      prevDirection = direction;
                    } else if (prevSpeed != speed) {
                      xhr.open("GET", `/speed?value=${speed}`, true);
                      xhr.send();
                      canSend = false;
                      prevSpeed = speed;
                    } else if (prevDirection != direction) {
                      xhr.open("GET", `/direction?value=${direction}`, true);
                      xhr.send();
                      canSend = false;
                      prevDirection = direction;
                    }
                  }
                }

                drawJoystick();
                ticks++;
            };
            var closeXHR = new XMLHttpRequest();
            window.onmouseup = () => {
              if (ticks == 0)
                return; 
              x = w / 2;
              y = h / 2;
              drawJoystick();
              
              speedLabel.innerHTML = "Speed: 0";
              speedSlider.value = 0;

              for (let i = 0; i < 5; i++) {
                closeXHR.open("GET", "/speed?value=0", true);
                closeXHR.send();
              }

              ticks = 0;
            }
            document.addEventListener('touchmove', function(e) {window.onmousemove(e)}, false);
            document.addEventListener('touchend', function(e) {window.onmouseup(e)}, false);
        </script>

        <p id="mode-label"></p>
        <iframe src='/dynamic_values' width='300' height='300' name='DataBox' id='values-box'></iframe>
        <canvas id="heading-canvas" width="300" height="300"/>
        <p id="tof-label"></p>

        <script type="text/javascript">
          let values = document.getElementById("values-box");
          values.style.display = "none";
          let headingCanvas = document.getElementById("heading-canvas");
          let headingRect = headingCanvas.getBoundingClientRect();
          let headingWidth = headingRect.width;
          let headingHeight = headingRect.height;
          let heading = 0;
          let headingContext = headingCanvas.getContext("2d");

          let tofLabel = document.getElementById("tof-label");
          let modeLabel = document.getElementById("mode-label");
          
          function update(event) {
            console.log(event);
            let page = values.contentWindow.document.body.innerHTML;
            if (!page)
              return;
            let data = JSON.parse(page);
            heading = data.heading;
            tofLabel.innerHTML = "TOFs: " + data.tofs;
            modeLabel.innerHTML = "Mode: " + data.mode;
            drawHeading()
          }

          function drawHeading() {
              rHeading = (-90 - heading) * Math.PI / 180;
              headingContext.fillStyle = 'rgb(255, 255, 255)';
              headingContext.fillRect(0, 0, 500, 500);

              headingContext.beginPath();
              headingContext.arc(headingWidth / 2, headingHeight / 2, 100, 0.14 * Math.PI + rHeading, 1.86 * Math.PI + rHeading, false);
              headingContext.lineWidth = 5;
              headingContext.strokeStyle = 'rgb(50, 50, 50)';
              headingContext.stroke();

              headingContext.translate(headingWidth / 2, headingHeight / 2);
              headingContext.rotate(rHeading);
              headingContext.fillStyle = 'rgb(50, 50, 50)';
              headingContext.strokeRect(-40, -25, 80, 50);
              headingContext.strokeRect(52, -45, 33, 90);
              headingContext.fillStyle = 'rgb(255, 255, 255)';
              headingContext.fillRect(70, -43, 20, 86);
              headingContext.setTransform(1, 0, 0, 1, 0, 0);
          }
          drawHeading();

          values.onload = update;
        </script>
      </body>
    </html>
  )rawliteral";

  return index_html;
}