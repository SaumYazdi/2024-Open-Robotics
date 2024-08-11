
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
    
    colorSwitch.checked = false;
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
      
