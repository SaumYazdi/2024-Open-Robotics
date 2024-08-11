
let distEquationLabel = getE("equation-dist-label");

let calibrateDistStart = getE("calibrate-dist-start");
let distOutputGraph = getE("output-dist-graph");
distOutputGraph.style.display = "none";

let calibrationDistFrame = getE("calibration-dist");
let calibrateDistLabel = getE("calibrate-dist-label");

steps = [];
step = 0;

function distCurveFit() {
    radii = steps.map(point => point.x);
    distance = steps.map(point => point.y);
    return fetch("/fitDist", {
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

function calcDist(radius) {
    return k * Math.pow(radius, a);
}

function distOutput() {
    response = distCurveFit();
    response.then(responseData => {
        let graph = responseData.graph;
        a = responseData.a;
        k = responseData.k;

        updateDistEquation(k, a);

        distOutputGraph.src = graph;
        calibrateDistLabel.innerHTML = "";
        measurements.style = "";
        distOutputGraph.style.display = "block";
        
        steps = [];
        step = 0;
        updating = true;
    })
    .catch(error => {return 0;});
}
function calibrateDistNext() {
    if (step === 0) {
        calibrateDistStart.style.display = "none";
        calibrationDistFrame.style.display = "flex";
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
        calibrateDistStart.style = "";
        calibrationDistFrame.style = "";
        setTimeout("distOutput();", 500);
    } else
        calibrateDistLabel.innerHTML = `Click the 'Next' button with the ball <b>${step}cm</b> away from the camera.`;
}

function updateDistEquation(_k, _a) {
    k = _k;
    a = _a;
    _k = round(_k, 2);
    _a = round(_a, 2);
    distEquationLabel.innerHTML = `Equation: ${_k}x^${_a}`;
}
