
let colorsLabel = getE("colors");
let colorSwitch = getE("color-select");

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

function clearSelectedColours() {
    selectedColors = [];
    colorsLabel.innerHTML = `Selecting Colours: ${colorSwitch.checked}`;
}

function colorSelect() {
    clearSelectedColours();
    if (colorSwitch.checked) {
        setPreview();
        preview.style.display = "block";
        previewStream.style.display = "none";
    } else {
        preview.style = "";
        previewStream.style = "";
    }
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
    .then(response => response.json())
    .then(data => {
        getColors();
    })
    .catch(error => {return 0;});
    
    colorSwitch.checked = false;
    colorSelect();
}

var selectedColors = [];
var pixelColor;
var imageData;
function setPreview() {
    if (visible == false)
        return null;
        
    let img = new window.Image();
    img.crossOrigin = `Anonymous`;
    
    img.src = previewStream.src;
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
            // previewContext.fillStyle = "white";
            // previewContext.fillRect(0, preview.height - 50, 50, 50);
            // previewContext.fillStyle = "rgb" + pixelColor;
            // previewContext.fillRect(5, preview.height - 45, 40, 40);
        };
    };
}

preview.addEventListener("click", () => {
    if (!colorSwitch.checked)
        return;
    selectedColors.push(pixelColor);
    colorsLabel.innerHTML = `Selecting Colours: true, Colors: ${selectedColors.join(', ')}`;   
});
      


// Color calibration

colorLower = {
    r: getE("color-lower-r"),
    g: getE("color-lower-g"),
    b: getE("color-lower-b")
}

colorUpper = {
    r: getE("color-upper-r"),
    g: getE("color-upper-g"),
    b: getE("color-upper-b")
}

colorLowerBox = getE("color-lower-box");
colorUpperBox = getE("color-upper-box");

function getColors() {
    fetch("/colors", {
        method: "POST",
        headers: {
            "Content-type": "application/json; charset=UTF-8"
        }
        })
        .then(response => response.json())
        .then(data => {
            lower = data.lower;
            colorLower.r.value = lower[0];
            colorLower.g.value = lower[1];
            colorLower.b.value = lower[2];
            upper = data.upper;
            colorUpper.r.value = upper[0];
            colorUpper.g.value = upper[1];
            colorUpper.b.value = upper[2];
            updateColor(false);
        })
        .catch(error => {return 0;});
}

window.addEventListener("load", () => {
    getColors();
});

function HSVtoRGB(h, s, v) {
    var r, g, b, i, f, p, q, t;
    if (arguments.length === 1) {
        s = h.s, v = h.v, h = h.h;
    }
    i = Math.floor(h * 6);
    f = h * 6 - i;
    p = v * (1 - s);
    q = v * (1 - f * s);
    t = v * (1 - (1 - f) * s);
    switch (i % 6) {
        case 0: r = v, g = t, b = p; break;
        case 1: r = q, g = v, b = p; break;
        case 2: r = p, g = v, b = t; break;
        case 3: r = p, g = q, b = v; break;
        case 4: r = t, g = p, b = v; break;
        case 5: r = v, g = p, b = q; break;
    }
    return {
        r: Math.round(r * 255),
        g: Math.round(g * 255),
        b: Math.round(b * 255)
    };
}

function updateColor(setting = true) {
    lower = [colorLower.r.value, colorLower.g.value, colorLower.b.value];
    upper = [colorUpper.r.value, colorUpper.g.value, colorUpper.b.value];
    
    if (setting) {
        fetch("/setColors", {
            method: "POST",
            body: JSON.stringify({
                lower: lower,
                upper: upper
            }),
            headers: {
                "Content-type": "application/json; charset=UTF-8"
            }
            })
            .then(response => response.json())
            .catch(error => {return 0;});
    }
    
    l = HSVtoRGB(lower[0] / 360, lower[1] / 255, lower[2] / 255)
    colorLowerBox.style.backgroundColor = `rgb(${l.r}, ${l.g}, ${l.b})`;
    u = HSVtoRGB(upper[0] / 360, upper[1] / 255, upper[2] / 255)
    colorUpperBox.style.backgroundColor = `rgb(${u.r}, ${u.g}, ${u.b})`;
}
