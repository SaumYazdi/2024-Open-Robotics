import numpy as np
from scipy.optimize import curve_fit

from flask import Flask, render_template, jsonify, request
import base64
import cv2

class Server(Flask):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._define_routes()
        
        self.radius = 0
        self.preview = None

    def _define_routes(self):
        @self.route("/")
        def index():
            return render_template("index.html")
            
        @self.route("/radius", methods=["POST"])
        def radius():
            return jsonify({"radius": self.radius})

        @self.route("/preview", methods=["POST"])
        def preview():
            if self.preview is None:
                return
            
            _, img_arr = cv2.imencode(".png", self.preview)
            img_bytes = img_arr.tobytes()
            image = "data:image/png;base64,"
            image += base64.b64encode(img_bytes).decode("utf-8")
            return jsonify({"preview": image})
        
        def func(x, k, a):
            return k / pow(x, a)
        
        @self.route("/fit", methods=["POST"])
        def fit():
            data = request.data()

            x_data = data["radii"] #[178, 106, 67, 53.5, 44.6, 42]
            y_data = data["distance"] #[10, 20, 30, 40, 50, 60]

            # Set initial guess to apparent inflection point
            initial_guess = [2000.0, -1.0]
            params, covariance = curve_fit(func, x_data, y_data, p0=initial_guess)

            # Extract the parameters
            k, a = params

            # # Create a range of x values for the curve change value of "127" to max number or data points i didnt know how to get max size of the data sheet
            # x_fit = np.linspace(min(x_data), max(x_data), 137)
            # # Calculate the y values for the fitted curve
            # y_fit = func(x_fit, k, a)

            return jsonify({k: k, a: a})

    def start(self):
        self.run(port=8080, host="0.0.0.0")
