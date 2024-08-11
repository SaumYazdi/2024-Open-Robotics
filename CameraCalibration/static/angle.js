
let angleEquationLabel = getE("equation-angle-label");

let calibrateAngleStart = getE("calibrate-angle-start");
let angleOutputGraph = getE("output-angle-graph");
angleOutputGraph.style.display = "none";

let calibrationAngleFrame = getE("calibration-angle");
let calibrateAngleLabel = getE("calibrate-angle-label");

steps = [];
step = -40; // Initial x-offset is -30cm

function angleCurveFit() {
    let xOffset = steps.map(point => point.x);
    let trueX = steps.map(point => point.y);
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

        angleOutputGraph.src = graph;
        calibrateAngleLabel.innerHTML = "";
        measurements.style = "";
        angleOutputGraph.style.display = "block";
        
        steps = [];
        step = -40;
        updating = true;
    })
    .catch(error => {return 0;});
}
function calibrateAngleNext() {
    if (step === -40) {
        calibrateAngleStart.style.display = "none";
        calibrationAngleFrame.style.display = "flex";
        measurements.style.display = "none";
        updating = false;
    } else {
        let originalStep = step;
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
                        steps.push({x: x / distance, y: originalStep});
                    })
                    .catch(error => {return 0;});
            })
            .catch(error => {return 0;});
    }

    step += 10;
    
    if (step > 30) {
        calibrateAngleStart.style = "";
        calibrationAngleFrame.style = "";
        setTimeout("angleOutput();", 500);
    } else {
            if (step == 0)
                calibrateAngleLabel.innerHTML = `Click the 'Next' button with the ball in front of the camera.`;
            else {
                let dir = step < 0 ? "left" : "right";
                calibrateAngleLabel.innerHTML = `Click the 'Next' button with the ball <b>${Math.abs(step)}cm</b> ${dir} from the camera.`;
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
