
let distEquationLabel = getE("equation-dist-label");
let distGraph = getE("dist-graph");

let calibrateDistStart = getE("calibrate-dist-start");

let calibrationDistFrame = getE("calibration-dist");
let calibrateDistLabel = getE("calibrate-dist-label");

distSteps = [];
distStep = 0;

function distCurveFit() {
    radii = distSteps.map(point => point.x);
    distance = distSteps.map(point => point.y);
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

        distGraph.src = graph;
        calibrateDistLabel.innerHTML = "";
        measurementsFrame.style = "";
        
        distSteps = [];
        distStep = 0;
        updating = true;
    })
    .catch(error => {return 0;});
}
function calibrateDistNext() {
    if (distStep === 0) {
        calibrateDistStart.style.display = "none";
        calibrationDistFrame.style.display = "flex";
        measurementsFrame.style.display = "none";
        updating = false;
    } else {
        let originalStep = distStep;
        fetch("/radius", {
            method: "POST",
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
            })
            .then(response => response.json())
            .then(data => {
                distSteps.push({x: data.radius * data.radialDistance, y: originalStep});
            })
            .catch(error => {return 0;});
    }

    distStep += 10;
    
    if (distStep > 60) {
        calibrateDistStart.style = "";
        calibrationDistFrame.style = "";
        setTimeout("distOutput();", 500);
    } else
        calibrateDistLabel.innerHTML = `Click the 'Next' button with the ball <b>${distStep}cm</b> away from the camera.`;
}

function updateDistEquation(_k, _a) {
    k = _k;
    a = _a;
    _k = round(_k, 2);
    _a = round(_a, 2);
    distEquationLabel.innerHTML = `Equation: ${_k}x^${_a}`;
}
