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
  bot = new Bot();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_IP, gateway, subnet); // Configure static IP
   
  // Setting the AP Mode with SSID, Password, and Max Connection Limit
  WiFi.softAP(ap_ssid,ap_password,1,false,max_connections);
 
  // Specifying the functions which will be executed upon corresponding GET request from the client
  server.on("/", HTTP_GET, handle_OnConnect);
  server.on("/speed", handle_speed);
  server.on("/turn", handle_turn);
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

void handle_turn() {
  bot->turnSpeed = server.arg("value").toInt();
  server.send(200, "text/html", HTML());
}

void handle_direction() {
  direction = server.arg("value").toInt();
  server.send(200, "text/html", HTML());

  bot->direction = direction;
}

void handle_NotFound() {
  server.send(404, "text/plain", "Not found");
}

String add_item(String data, String key, String value) {
  return data + "\"" + key + "\": " + value; 
}

String convertToString(int* a, int size)
{
    String s = "";
    for (int i = 0; i < size; i++) {
      s = s + String(a[i]);
      if (i < size - 1) {
        s += ", ";
      }
    }
    return s;
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

  String json_data = "{";
  json_data = add_item(json_data, "heading", heading) + ",";
  json_data = add_item(json_data, "tofs", distances) + ",";
  json_data = add_item(json_data, "distances", "\"" + convertToString(bot->logic.distances, 8) + "\"") + ",";
  json_data = add_item(json_data, "simDistances", "\"" + convertToString(bot->logic.simDistances, 8) + "\"") + ",";
  json_data = add_item(json_data, "mode", "\"" + mode + "\"") + ",";
  json_data = add_item(json_data, "x", String(bot->logic.robotX)) + ",";
  json_data = add_item(json_data, "y", String(bot->logic.robotY)) + ",";
  json_data = add_item(json_data, "kickoffTicks", String(bot->logic.kickoffTicks)) + ",";
  json_data = add_item(json_data, "lostTicks", String(bot->logic.lostTicks)) + ",";
  json_data = add_item(json_data, "distance", String(bot->logic.ballDistance)) + ",";
  json_data = add_item(json_data, "angle", String(bot->logic.ballAngle));
  json_data += "}";

  server.send(200, "text/plain", json_data);
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
        <h1 id="title">Robot Controller</h1>
        <h3 id="subtitle">Using Access Point (AP) Mode</h3>
        
        <button onclick="location.reload()">Refresh</button>

        <p id="mode-label"></p>
        <canvas id="heading-canvas" width="1280" height="1280"></canvas>
        <div style="position: fixed; right: 0; bottom: 0;">
          <button id="slow-left" style="margin: 1rem; user-select: none; -webkit-user-select: none;"><</button>
          <canvas id="joystick" width="210" height="210"></canvas>
          <button id="slow-right" style="margin: 1rem; user-select: none; -webkit-user-select: none;">></button>
        </div>
        <script type="text/javascript">
          let slowLeftButton = document.getElementById("slow-left");
          let slowRightButton = document.getElementById("slow-right");
          let turnXHR = new XMLHttpRequest();
          slowLeftButton.addEventListener("touchstart", () => {turnXHR.open("GET", "/turn?value=-10000", true); turnXHR.send(null);});
          slowLeftButton.addEventListener("touchend", () => {turnXHR.open("GET", "/turn?value=0", true); turnXHR.send(null);});
          slowRightButton.addEventListener("touchstart", () => {turnXHR.open("GET", "/turn?value=10000", true); turnXHR.send(null);});
          slowRightButton.addEventListener("touchend", () => {turnXHR.open("GET", "/turn?value=0", true); turnXHR.send(null);});
        </script>
        <script type="text/javascript">
          let headingCanvas = document.getElementById("heading-canvas");
          let headingRect = headingCanvas.getBoundingClientRect();
          let headingWidth = headingRect.width;
          let headingHeight = headingRect.height;

          let subTitle = document.getElementById("subtitle");

          let ww = window.innerWidth * 0.9;
          headingCanvas.style.width = ww + "px";
          headingCanvas.style.height = ww * headingHeight / headingWidth + "px";

          let heading = 0;
          let tofs = JSON.parse('{"-33.50": 0, "-68.50": 0, "-113.50": 0, "-158.50": 0, "158.50": 0, "113.50": 0, "68.50": 0, "33.50": 0}');
          let simDistances = tofs;
          let headingContext = headingCanvas.getContext("2d");

          let modeLabel = document.getElementById("mode-label");
          
          let ballDistance = null;
          let ballAngle = null;

          let dataReq = new XMLHttpRequest();
          dataReq.onload = update;
          dataReq.open("GET", "/dynamic_values", true);
          dataReq.send(null);
          function update(event) {
            page = dataReq.responseText;
            let data = JSON.parse(page);
            heading = data.heading;
            tofs = data.tofs;
            simDistances = data.simDistances.split(", ");
            modeLabel.innerHTML = "Mode: " + data.mode;
            drawHeading();

            subTitle.innerHTML = `TOF Distance: ${data.distances} <br>Simulated Distance: ${data.simDistances} <br>Distance: ${data.distance} <br>Angle: ${data.angle} <br>Position: ${x}, ${y}`;
            ballAngle = parseInt(data.angle) * Math.PI / 180;
            ballDistance = parseInt(data.distance);

            dataReq.open("GET", "/dynamic_values", true);
            dataReq.send(null);
          }

          let headingRadius = 50;
          let distanceScaleFactor = .2;
          let rHeading, rAngle, prevRot, diff, distance;
          function drawHeading() {
            let scale = headingRadius / 100;
            rHeading = (-90 - heading) * Math.PI / 180;
            headingContext.fillStyle = 'rgb(255, 255, 255)';
            headingContext.fillRect(0, 0, headingWidth, headingHeight);

            headingContext.beginPath();
            headingContext.arc(headingWidth / 2, headingHeight / 2, headingRadius, 0.14 * Math.PI + rHeading, 1.86 * Math.PI + rHeading, false);
            headingContext.lineWidth = 5 * scale;
            headingContext.strokeStyle = 'rgb(50, 50, 50)';
            headingContext.stroke();

            headingContext.translate(headingWidth / 2, headingHeight / 2);
            headingContext.rotate(rHeading);
            headingContext.fillStyle = 'rgb(50, 50, 50)';
            headingContext.strokeRect(-40 * scale, -25 * scale, 80 * scale, 50 * scale);
            headingContext.strokeRect(52 * scale, -45 * scale, 33 * scale, 90 * scale);
            headingContext.fillStyle = 'rgb(255, 255, 255)';
            headingContext.fillRect(70 * scale, -43 * scale, 20 * scale, 86 * scale);

            prevRot = 0;
            let i = 0;
            for (let tofAngle in tofs) {
              distance = tofs[tofAngle] * distanceScaleFactor;
              if (distance > 0 && distance <= 10000 * distanceScaleFactor) {
                rAngle = tofAngle * Math.PI / 180;
                diff = rAngle - prevRot;
                headingContext.rotate(diff);

                headingContext.strokeStyle = 'rgb(50, 50, 50)';
                headingContext.beginPath();
                headingContext.moveTo(headingRadius, 0);
                headingContext.lineTo(headingRadius + distance, 0);
                headingContext.stroke();

                prevRot = rAngle;
              }

              headingContext.beginPath();
              headingContext.arc(headingRadius + parseInt(simDistances[i]), 0, 5, 0, 2 * Math.PI, false);
              headingContext.fillStyle = 'rgb(110, 70, 230)';
              headingContext.fill();

              i++;
            }

            let ballRadius = 12 * scale;
            let ballDistanceScale = 6 * scale;
            headingContext.beginPath();
            let bx = ballDistanceScale * ballDistance * Math.cos(ballAngle);
            let by = ballDistanceScale * ballDistance * Math.sin(ballAngle);
            headingContext.arc(bx, by, ballRadius, 0, 2 * Math.PI, false);
            headingContext.fillStyle = 'rgb(250, 50, 5)';
            headingContext.fill();

            headingContext.setTransform(1, 0, 0, 1, 0, 0);
          }
          drawHeading();
        </script>

        <script type="text/javascript">
            var canvas = document.getElementById("joystick");
            let rect = canvas.getBoundingClientRect();
            let w = rect.width;
            let h = rect.height;
            let x = w / 2;
            let y = h / 2;
            let joystickRadius = 70;
            let innerRadius = 30;

            var previewContext = canvas.getContext("2d");
            previewContext.lineWidth = 3;
            
            function drawJoystick(ctx) {
                ctx.clearRect(0, 0, w, h);

                ctx.beginPath();
                ctx.arc(w / 2, h / 2, joystickRadius, 0, 2 * Math.PI, false);
                ctx.strokeStyle = 'rgb(50, 50, 50)';
                ctx.stroke();

                ctx.beginPath();
                ctx.arc(x, y, innerRadius, 0, 2 * Math.PI, false);
                ctx.fillStyle = 'rgb(120, 120, 120)';
                ctx.fill();
                ctx.strokeStyle = 'rgb(50, 50, 50)';
                ctx.stroke();
            }
            drawJoystick(previewContext);

            var xhr = new XMLHttpRequest();
            xhr.addEventListener("load", (event) => {canSend = true;});
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

                if (dist > joystickRadius) {
                    if (ticks == 0)
                        return;
                    x = w / 2 - Math.cos(direction) * joystickRadius;
                    y = h / 2 - Math.sin(direction) * joystickRadius;
                }
                direction = direction * 180 / Math.PI - 90;
                let speed = Math.min(maxSpeed, dist * maxSpeed / joystickRadius);

                speed = Math.round(speed);
                direction = Math.round(direction) % 360;

                if (canSend) {
                  if (prevSpeed != speed && prevDirection != direction) {
                    xhr.open("GET", `/update?speed=${speed}&direction=${direction}`, true);
                    xhr.send();
                    prevSpeed = speed;
                    prevDirection = direction;
                    canSend = false;
                  } else if (prevSpeed != speed) {
                    xhr.open("GET", `/speed?value=${speed}`, true);
                    xhr.send();
                    prevSpeed = speed;
                    canSend = false;
                  } else if (prevDirection != direction) {
                    xhr.open("GET", `/direction?value=${direction}`, true);
                    xhr.send();
                    prevDirection = direction;
                    canSend = false;
                  }
                }

                drawJoystick(previewContext);
                ticks++;
            };
            var closeXHR = new XMLHttpRequest();
            window.onmouseup = () => {
                
              x = w / 2;
              y = h / 2;
              drawJoystick(previewContext);
              
              for (let i = 0; i < 4; i++) {
                closeXHR.open("GET", "/speed?value=0", true);
                closeXHR.send();
              }

              ticks = 0;
            }
            document.addEventListener('touchmove', function(e) {window.onmousemove(e)}, false);
            document.addEventListener('touchend', function(e) {window.onmouseup(e)}, false);
        </script>
      </body>
    </html>
  )rawliteral";

  return index_html;
}

