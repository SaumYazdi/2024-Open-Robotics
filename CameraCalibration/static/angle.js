
let angleEquationLabel = getE("equation-angle-label");
let angleGraph = getE("angle-graph");

let calibrateAngleStart = getE("calibrate-angle-start");

let calibrationAngleFrame = getE("calibration-angle");
let calibrateAngleLabel = getE("calibrate-angle-label");

angleSteps = [];
angleStep = -40; // Initial x-offset is -30cm

function angleCurveFit() {
    let xOffset = angleSteps.map(point => point.x);
    let trueX = angleSteps.map(point => point.y);
    return fetch("/fitAngle", {
        method: "POST",
        body: JSON.stringify({
            x: xOffset,
            trueX: trueX
        }),
        headers: {
            "Content-type": "application/json; charset=UTF-8"
        }
        })
        .then(response => response.json())
}

function calcX(xOffset) {
    return parseFloat(m * (xOffset / distance)) + parseFloat(c);
}

function calcAngle(x) {
    return Math.atan2(x, distance);
}

function angleOutput() {
    response = angleCurveFit();
    response.then(responseData => {
        let graph = responseData.graph;
        m = responseData.m;
        c = responseData.c;

        updateAngleEquation(m, c);

        angleGraph.src = graph;
        calibrateAngleLabel.innerHTML = "";
        measurementsFrame.style = "";
        
        angleSteps = [];
        angleStep = -40;
        updating = true;
    })
    .catch(error => {return 0;});
}
function calibrateAngleNext() {
    if (angleStep === -40) {
        calibrateAngleStart.style.display = "none";
        calibrationAngleFrame.style.display = "flex";
        measurementsFrame.style.display = "none";
        updating = false;
    } else {
        let originalStep = angleStep;
        fetch("/xOffset", {
            method: "POST",
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
            })
            .then(response => response.json())
            .then(data => {
                let x = data.xOffset;
                fetch("/radius", {
                    method: "POST",
                    headers: {
                        "Content-type": "application/json; charset=UTF-8"
                    }
                    })
                    .then(response => response.json())
                    .then(data => {
                        radius = data.radius;
                        distance = calcDist(radius);
                        console.log(radius, distance);
                        angleSteps.push({x: x / distance, y: originalStep});
                    })
                    .catch(error => {return 0;});
            })
            .catch(error => {return 0;});
    }

    angleStep += 10;
    
    if (angleStep > 30) {
        calibrateAngleStart.style = "";
        calibrationAngleFrame.style = "";
        setTimeout("angleOutput();", 500);
    } else {
            if (angleStep == 0)
                calibrateAngleLabel.innerHTML = `Click the 'Next' button with the ball in front of the camera.`;
            else {
                let dir = angleStep < 0 ? "left" : "right";
                calibrateAngleLabel.innerHTML = `Click the 'Next' button with the ball <b>${Math.abs(angleStep)}cm</b> ${dir} from the camera.`;
        }
    }
}

function updateAngleEquation(_m, _c) {
    m = _m;
    c = _c;
    _m = round(_m, 2);
    _c = round(_c, 2);
    angleEquationLabel.innerHTML = `Equation: ${_m}x + ${_c}`;
}
