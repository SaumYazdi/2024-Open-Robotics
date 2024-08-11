

function getE(id) {
    return document.getElementById(id);
}

function round(value, digits) {
    let mult = Math.pow(10, digits);
    return Math.round(value * mult) / mult;
}

// Initialise HTML Elements
let measurements = getE("measurements");
let updateLabel = getE("update-label");
let updateSlider = getE("update-slider");

let previewFrame = getE("preview-frame");
let preview = getE("preview");
let previewButton = getE("preview-button");
let colorsLabel = getE("colors");
let colorSwitch = getE("color-select");

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
        previewFrame.style.display = "flex";
        visible = true;
    } else {
        previewButton.innerHTML = "Show Preview";
        previewFrame.style = "";
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

function extractPixelColor(cols, x, y) {
    let pixel = cols * parseInt(x) + parseInt(y);
    let position = pixel * 4;
    return {
        red: imageData[position],
        green: imageData[position + 1],
        blue: imageData[position + 2],
        alpha: imageData[position + 3],
    };
};

function colorSelect() {
    selectedColors = [];
    colorsLabel.innerHTML = `Selecting Colours: ${colorSwitch.checked}`;
}

function calibrateColors() {
    if (selectedColors.length == 0)
        return;
       
    fetch("/calibrateColors", {
        method: "POST",
        body: JSON.stringify(selectedColors),
        headers: {
            "Content-type": "application/json; charset=UTF-8"
        }
    })
    .catch(error => {return 0;});
}

var selectedColors = [];
var pixelColor;
var imageData;
function setPreview(src) {
    if (visible == false)
        return null;
        
    let img = new window.Image();
    img.crossOrigin = `Anonymous`;
    
    img.src = src;
    img.onload = function() {
    
        preview.width = img.width;
        preview.height = img.height;
        
        previewContext = preview.getContext("2d", willReadFrequently = true);
        previewContext.drawImage(img, 0, 0);
        
        imageData = previewContext.getImageData(0, 0, preview.width, preview.height).data;
        
        preview.onmousemove = (event) => {
            if (!colorSwitch.checked)
                return;
                
            let cols = preview.width;
            let rect = preview.getBoundingClientRect();
            let scaleX = preview.width / rect.width;
            let scaleY = preview.height / rect.height;
            let x = (event.clientX - rect.left) * scaleX;
            let y = (event.clientY - rect.top) * scaleY;
            
            let c = extractPixelColor(cols, y, x);
            pixelColor = `(${c.red}, ${c.green}, ${c.blue})`;
            previewContext.fillStyle = "white";
            previewContext.fillRect(0, preview.height - 50, 50, 50);
            previewContext.fillStyle = "rgb" + pixelColor;
            previewContext.fillRect(5, preview.height - 45, 40, 40);
        };
    };
}

preview.addEventListener("click", () => {
    if (!colorSwitch.checked)
        return;
    selectedColors.push(pixelColor);
    colorsLabel.innerHTML = `Selecting Colours: true, Colors: ${selectedColors.join(', ')}`;   
});
        
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
        if (!colorSwitch.checked) {
            fetch("/preview" , {
                method: "POST",
                headers: {
                    "Content-type": "application/json; charset=UTF-8"
                }
                })
                .then(response => response.json())
                .then(data => {
                    setPreview(data.preview);
                })
                .catch(error => {return 0;});
        }
    }

    ticks++;

    setTimeout("update();", 100);
}

update();
