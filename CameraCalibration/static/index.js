var radius = 0;

function getE(id) {
    return document.getElementById(id);
}

let detectedLabel = getE("detected-label");
let radiusLabel = getE("radius-label");
let distanceLabel = getE("distance-label");
let factorLabel = getE("factor-label");
let factorSlider = getE("factor-slider");

let measurements = getE("measurements");
let calibrateStart = getE("calibrate-start");
let outputLabel = getE("output-label");
let outputGraph = getE("output-graph");
outputGraph.style.display = "none";

let calibration = getE("calibration");
let calibrateLabel = getE("calibrate-label");
let updateLabel = getE("update-label");
let updateSlider = getE("update-slider");
let preview = getE("preview");
let previewButton = getE("preview-button");

UPDATE_INTERVAL = 200 //ms
updateLabel.innerHTML = `Update Interval: ${round(UPDATE_INTERVAL * .001, 3)}s`;
updateSlider.addEventListener("input", () => {
    UPDATE_INTERVAL = updateSlider.value;
    updateLabel.innerHTML = `Update Interval: ${round(UPDATE_INTERVAL * .001, 3)}s`;
});

var updating = true;
var visible = false;

factor = 2440;
factorLabel.innerHTML = `Factor: ${factor}`;
factorSlider.addEventListener("input", () => {
    factor = factorSlider.value;
    factorLabel.innerHTML = `Factor: ${factor}`;
});

steps = [];
step = 0;

function curveFit() {
    radii = steps.map(point => point.x);
    distance = steps.map(point => point.y);
    return fetch("/fit", {
        method: "POST",
        body: JSON.stringify({
            radii: radii,
            distance: distance
        }),
        headers: {
            "Content-type": "application/json; charset=UTF-8"
        }
        })
        .then(response => response.json())
}

function round(value, digits) {
    let mult = Math.pow(10, digits);
    return Math.round(value * mult) / mult;
}

function output() {
    outputLabel.innerHTML = steps.map(point => point.x + " " + point.y).join("<br>");
    
    response = curveFit();
    response.then(responseData => {
        let graph = responseData.graph;
        let a = responseData.a, k = responseData.k;
        let equation = `${round(k, 2)}x^${round(a, 2)}`;

        outputGraph.src = graph;
        outputLabel.innerHTML = steps.map(point => point.x + " " + point.y).join("<br>");
        outputLabel.innerHTML += `<b><br>Equation: ${equation}</b>`;
        calibrateLabel.innerHTML = "";
        measurements.style = "";
        outputGraph.style.display = "block";
        
        steps = [];
        step = 0;
        updating = true;
    })
    .catch(error => {return 0;});
}
function calibrateNext() {
    if (step === 0) {
        calibrateStart.style.display = "none";
        calibration.style.display = "flex";
        measurements.style.display = "none";
        updating = false;
    } else {
        let originalStep = step;
        fetch("/radius", {
            method: "POST",
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
            })
            .then(response => response.json())
            .then(data => {
                steps.push({x: data.radius, y: originalStep});
            })
            .catch(error => {return 0;});
    }

    step += 10;
    
    if (step > 60) {
        calibrateStart.style = "";
        calibration.style = "";
        setTimeout("output();", 500);
    } else
        calibrateLabel.innerHTML = `Click the 'Next' button with the ball <b>${step}cm</b> away from the camera.`;
}

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
    if (updating && (ticks % Math.round(1000 / UPDATE_INTERVAL) == 0)) {
        // Sends a fetch radius request every second regardless of update interval.
        fetch("/radius", {
            method: "POST",
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
            })
            .then(response => response.json())
            .then(data => {
                detectedLabel.innerHTML = `Detected: ${(data.radius !== null)}`
                radiusLabel.innerHTML = `Radius: ${data.radius}`;
                radius = data.radius;
                
                distanceLabel.innerHTML = `Estimated Distance: ${factor / radius}`;
            })
            .catch(error => {return 0;});
    }
    if (visible) {
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

    setTimeout("update();", UPDATE_INTERVAL);
}

update();
