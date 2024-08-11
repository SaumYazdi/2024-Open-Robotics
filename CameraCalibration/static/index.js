

function getE(id) {
    return document.getElementById(id);
}

function round(value, digits) {
    let mult = Math.pow(10, digits);
    return Math.round(value * mult) / mult;
}

let measurements = getE("measurements");
let updateLabel = getE("update-label");
let updateSlider = getE("update-slider");
let preview = getE("preview");
let previewButton = getE("preview-button");

let detectedLabel = getE("detected-label");
let radiusLabel = getE("radius-label");
let distanceLabel = getE("distance-label");
let xOffsetLabel = getE("x-label");
let angleLabel = getE("angle-label");

let measureLabel = getE("measure-label");
let measureSlider = getE("measure-slider");

UPDATE_INTERVAL = updateSlider.value;
updateLabel.innerHTML = `Update Interval: ${round(UPDATE_INTERVAL * .001, 3)}s`;
updateSlider.addEventListener("input", () => {
    UPDATE_INTERVAL = updateSlider.value;
    updateLabel.innerHTML = `Update Interval: ${round(UPDATE_INTERVAL * .001, 3)}s`;
});

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

// Parameters
var k, a;
var m, c;

var updating = true;
var visible = false;

function showPreview() {
    let text = previewButton.innerHTML;
    if (text == "Show Preview") {
        previewButton.innerHTML = "Hide Preview";
        preview.style.display = "block";
        visible = true;
    } else {
        previewButton.innerHTML = "Show Preview";
        preview.style = "";
        visible = false;
        fetch("hidePreview", {
                method: "POST",
                headers: {
                    "Content-type": "application/json; charset=UTF-8"
                }
            })
            .catch(error => {return 0;});
    }
}

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
                detectedLabel.innerHTML = `Detected: ${(radius !== null)}`
                radiusLabel.innerHTML = `Radius: ${radius}`;
                
                distance = calcDist(radius);
                distanceLabel.innerHTML = `Estimated Distance: ${distance}`;
            })
            .catch(error => {return 0;});
        // Fetches ball x-position on camera.
        fetch("/xOffset", {
            method: "POST",
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
            })
            .then(response => response.json())
            .then(data => {
                let x = data.xOffset;
                estimatedX = calcX(x);
                xOffsetLabel.innerHTML = `Estimated X: ${estimatedX}`;
                
                angle = calcAngle(estimatedX);
                angleLabel.innerHTML = `Estimated Angle: ${(angle * 180) / Math.PI}`;
            })
            .catch(error => {return 0;});
    }
    if (visible && (ticks % Math.round(UPDATE_INTERVAL / 100) == 0)) {
        // Retrieve camera image
        fetch("/preview" , {
            method: "POST",
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
            })
            .then(response => response.json())
            .then(data => {
                preview.src = data.preview;
            })
            .catch(error => {return 0;});
    }

    ticks++;

    setTimeout("update();", 100);
}

update();
