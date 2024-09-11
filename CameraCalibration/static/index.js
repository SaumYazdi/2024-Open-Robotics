function getE(id) {
    return document.getElementById(id);
}

function round(value, digits) {
    let mult = Math.pow(10, digits);
    return Math.round(value * mult) / mult;
}

// Initialise HTML Elements
let measurementsFrame = getE("measurements-frame");
let measurements = getE("measurements");

let fpsLabel = getE("fps");
let previewStream = getE("preview-stream");
let previewFrame = getE("preview-frame");
let preview = getE("preview");
let previewButton = getE("preview-button");

let detectedLabel = getE("detected-label");
let radiusLabel = getE("radius-label");
let distanceLabel = getE("distance-label");
let xOffsetLabel = getE("x-label");
let angleLabel = getE("angle-label");

let measureLabel = getE("measure-label");
let measureSlider = getE("measure-slider");

let fieldDisplay = getE("field-display");

let circleMask = getE("calibrate-circle-mask");

MEASURE_INTERVAL = measureSlider.value;
measureLabel.innerHTML = `Measure every ${round(MEASURE_INTERVAL * .001, 3)}s`;
measureSlider.addEventListener("input", () => {
    MEASURE_INTERVAL = measureSlider.value;
    measureLabel.innerHTML = `Measure every ${round(MEASURE_INTERVAL * .001, 3)}s`;
});

// Global variables
var radius = 0;
var distance = 0;
var estimatedX = 0;
var angle = 0;

var fps = 0;

// Parameters
var k, a;
var m, c;

var updating = true; // Preview updates
var visible = false; // Preview visibility

function showPreview() {
    let text = previewButton.innerHTML;
    if (text == "Show Preview") {
        // createOffer();
        previewStream.src = "/preview";
        previewButton.innerHTML = "Hide Preview";
        previewFrame.style.display = "flex";
        visible = true;
    } else {
        previewStream.src = "";
        previewButton.innerHTML = "Show Preview";
        previewFrame.style = "";
        visible = false;
        fetch("/hidePreview", {
            method: "POST",
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
        })
        .catch(error => {return 0;});
    }
}
  
let context = fieldDisplay.getContext("2d");
let sw = fieldDisplay.width;
let sh = fieldDisplay.height;
let fw = 500;
        
ticks = 0;
function update() {
    if (updating && (ticks % Math.round(MEASURE_INTERVAL / 100) == 0)) {
        
        // Sends a fetch radius request every measure interval regardless of update interval.
        fetch("/radius", {
            method: "POST",
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
            })
            .then(response => response.json())
            .then(data => {
                radius = data.radius;
                if (radius !== null) {
                    detectedLabel.innerHTML = "Ball Detected";
                    measurements.style.display = "flex";
                } else {
                    detectedLabel.innerHTML = "Ball Not Detected";
                    measurements.style = "";
                }
                radiusLabel.innerHTML = `Radius: ${Math.round(radius)}`;
                
            })
            .catch(error => {return 0;});
            
        fetch("/distance", {
            method: "POST",
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
            })
            .then(response => response.json())
            .then(data => {
                distance = data.distance;
                distanceLabel.innerHTML = `Estimated Distance: ${Math.round(distance * 100) / 100} cm`;
            })
            .catch(error => {return 0;});
            
        // Fetch ball angle from camera center
        fetch("/angle", {
            method: "POST",
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
            })
            .then(response => response.json())
            .then(data => {
                angle = data.angle;
                angleLabel.innerHTML = `Estimated Angle: ${Math.round((angle * 18000) / Math.PI) / 100} deg`;
            })
            .catch(error => {return 0;});
        
        if (visible) {
        // Retrive system FPS.
        fetch("/fps", {
            method: "POST",
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
            })
            .then(response => response.json())
            .then(data => {
                fps = data.fps;
                fpsLabel.innerHTML = `${fps} FPS`;
            })
            .catch(error => {return 0;});
        }
            
        context.clearRect(0, 0, sw, sh);
        
        context.beginPath();
        context.arc(sw / 2, sh / 2, 10 * fw / sw, 0, 2 * Math.PI, false);
        context.fillStyle = '#42f5aa';
        context.fill();
        context.lineWidth = 2;
        context.strokeStyle = '#51ad92';
        context.stroke();
        
        w = 10 * fw / sw;
        var path = new Path2D();
        path.moveTo(sw / 2 + w, sh / 2 + 5);
        path.lineTo(sw / 2 + w + 10, sh / 2);
        path.lineTo(sw / 2 + w, sh / 2 - 5);
        context.fill(path);
        
        if (distance && angle) {
            context.beginPath();
            let ball_pos = [sw/2 + Math.cos(angle) * distance * fw / sw, sh/2 - Math.sin(angle) * distance * fw / sw]
            context.arc(ball_pos[0], ball_pos[1], 5 * fw / sw, 0, 2 * Math.PI, false);
            context.fillStyle = 'orange';
            context.fill();
            context.lineWidth = 2;
            context.strokeStyle = '#f56642';
            context.stroke();
            
            context.font = "11px Arial";
            context.fillStyle = "#4265cf";
            context.fillText(`${Math.round(angle * 1800 / Math.PI) / 10} deg`, ball_pos[0] - 30, ball_pos[1] + 20);
            context.fillText(`${Math.round(distance * 10) / 10} cm`, ball_pos[0] - 20, ball_pos[1] + 32);
        }
    }

    ticks++;

    setTimeout("update();", 100);
}

update();

previewStream.addEventListener("load", () => {
    
    // Retrive circle mask radius.
    fetch("/maskRadius", {
        method: "POST",
        headers: {
            "Content-type": "application/json; charset=UTF-8"
        }
    })
    .then(response => response.json())
    .then(data => {
        circleMask.value = data.radius;
    })
    .catch(error => {return 0;});
        
    setPreview();
});

circleMask.addEventListener("change", () => {
    let maskRadius = circleMask.value;
    fetch("/setMaskRadius", {
        method: "POST",
        body: JSON.stringify({radius: maskRadius}),
        headers: {
            "Content-type": "application/json; charset=UTF-8"
        }
    })
    .catch(error => {return 0;});
});