
let angleEquationLabel = getE("equation-angle-label");

let calibrateAngleStart = getE("calibrate-angle-start");
let angleOutputGraph = getE("output-angle-graph");
angleOutputGraph.style.display = "none";

let calibrationAngleFrame = getE("calibration-angle");
let calibrateAngleLabel = getE("calibrate-angle-label");

var k, a;
var radius = 0;

steps = [];
step = 0;

function angleCurveFit() {
    radii = steps.map(point => point.x);
    angle = steps.map(point => point.y);
    return fetch("/fitAngle", {
        method: "POST",
        body: JSON.stringify({
            radii: radii,
            angle: angle
        }),
        headers: {
            "Content-type": "application/json; charset=UTF-8"
        }
        })
        .then(response => response.json())
}

function calcAngle(radius) {
    return k * Math.pow(radius, a);
}

function angleOutput() {
    response = angleCurveFit();
    response.then(responseData => {
        let graph = responseData.graph;
        a = responseData.a, k = responseData.k;
        let equation = `${round(k, 2)}x^${round(a, 2)}`;

        updateAngleEquation(k, a);

        angleOutputGraph.src = graph;
        calibrateAngleLabel.innerHTML = "";
        measurements.style = "";
        angleOutputGraph.style.display = "block";
        
        steps = [];
        step = 0;
        updating = true;
    })
    .catch(error => {return 0;});
}
function calibrateAngleNext() {
    if (step === 0) {
        calibrateAngleStart.style.display = "none";
        calibrationAngleFrame.style.display = "flex";
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
        calibrateAngleStart.style = "";
        calibrationAngleFrame.style = "";
        setTimeout("angleOutput();", 500);
    } else
        calibrateAngleLabel.innerHTML = `Click the 'Next' button with the ball <b>${step}cm</b> away from the camera.`;
}

function updateAngleEquation(_k, _a) {
    _k = round(_k, 2);
    _a = round(_a, 2);
    angleEquationLabel.innerHTML = `Equation: ${_k}x^${_a}`;
    k = _k;
    a = _a;
}
