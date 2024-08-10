

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

UPDATE_INTERVAL = 200 //ms
updateLabel.innerHTML = `Update Interval: ${round(UPDATE_INTERVAL * .001, 3)}s`;
updateSlider.addEventListener("input", () => {
    UPDATE_INTERVAL = updateSlider.value;
    updateLabel.innerHTML = `Update Interval: ${round(UPDATE_INTERVAL * .001, 3)}s`;
});

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
                radius = data.radius;
                detectedLabel.innerHTML = `Detected: ${(radius !== null)}`
                radiusLabel.innerHTML = `Radius: ${radius}`;
                
                distanceLabel.innerHTML = `Estimated Distance: ${calcDist(radius)}`;
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
