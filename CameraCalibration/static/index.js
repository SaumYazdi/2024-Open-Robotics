var radius = 0;
        
let radiusLabel = document.getElementById("radius-label");
let distanceLabel = document.getElementById("distance-label");
let factorLabel = document.getElementById("factor-label");
let factorSlider = document.getElementById("factor-slider");

let measurements = document.getElementById("measurements");
let calibrateStart = document.getElementById("calibrate-start");
let outputLabel = document.getElementById("output-label");
let outputGraph = document.getElementById("output-graph");
outputGraph.style.display = "none";
let calibration = document.getElementById("calibration");
let calibrateLabel = document.getElementById("calibrate-label");
let preview = document.getElementById("preview");

updating = true;

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

Chart.register({
    id: "functions",
    beforeInit: (chart, args, opts) => {
        var data = chart.config.data;
        for (var i = 0; i < data.datasets.length; i++) {
            if (data.datasets[i].function) {
                for (var j = 0; j < data.labels.length; j++) {
                    var fct = data.datasets[i].function,
                        x2 = data.labels[j],
                        y = fct(x2);
                    data.datasets[i].data.push(y);
                }
            }
        }
    }
});

function round(value, digits) {
    let mult = Math.pow(10, digits);
    return Math.round(value * mult) / mult;
}

function output() {
    outputLabel.innerHTML = steps.map(point => point.x + " " + point.y).join("<br>");
    
    response = curveFit();
    response.then(responseData => {
        let points = steps.map(point => point);
        let xData = steps.map(point => point.x);
        let min = Math.min(...xData);
        let max = Math.max(...xData);
        let increment = (max - min) / 10;
        let labels = [];
        for (let x = min; x < max; x += increment)
            labels.push(round(x, 2));
        
        let k = responseData.k, a = responseData.a;
        console.log(k, a);
        var graphData = {
            labels: labels,
            datasets: [{
                type: "scatter",
                label: "Data",
                data: points,
                pointRadius: 4,
                pointBackgroundColor: "rgba(0, 0, 255, 1)"
            },
            {
                type: "line",
                label: `Prediction (d = ${round(k, 2)}x^${round(a, 2)})`,
                function: function(x) { return k * Math.pow(x, a) },
                data: [],
                borderColor: "rgba(153, 102, 255, 1)",
                fill: false,
                xAxisID: 'x2'
            }]
        };
        graph = new Chart(outputGraph, {
            type: "scatter",
            data: graphData,
            options: {
                scales: {
                  x: {
                    //min: 0,
                    //max: 300,
                    ticks: {
                      stepSize: 10
                    }
                  },
                  x2: {
                    //min: 0,
                    //max: 120,
                    position: 'bottom',
                    type: 'category'
                  },
                  y: {
                    min: 5,
                    max: 65,
                    ticks: {
                      stepSize: 10
                    }
                    //grid: {
                      //display: false
                    //}
                  },
                }
            }
        });
        
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
        calibrateLabel.innerHTML = `Place the ball ${step}cm away from the camera.`;
}

function showPreview() {
    fetch("/preview", {
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

function update() {
    if (updating) {
        fetch("/radius", {
            method: "POST",
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
            })
            .then(response => response.json())
            .then(data => {
                radiusLabel.innerHTML = `Radius: ${data.radius}`;
                radius = data.radius;
                
                distanceLabel.innerHTML = `Estimated Distance: ${factor / radius}`;
            })
            .catch(error => {return 0;});
    }

    setTimeout("update();", 1000);
}

update();
