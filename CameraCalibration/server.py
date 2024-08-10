import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

from flask import Flask, render_template, jsonify, request
import base64
import cv2
import json

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
            return k * pow(x, a)
        
        @self.route("/fit", methods=["POST"])
        def fit():
            data = json.loads(request.data)

            x_data = data["radii"]
            y_data = data["distance"]

            initial_guess = [2000.0, -1.0]
            params, covariance = curve_fit(func, x_data, y_data, p0=initial_guess)

            k, a = params

            
            # Create a range of x values for the curve change value of "127" to max number or data points i didnt know how to get max size of the data sheet
            x_fit = np.linspace(min(x_data), max(x_data), 137)
            # Calculate the y values for the fitted curve
            y_fit = func(x_fit, k, a)
            
            # Plot of the data and the fitted curve
            plt.figure(figsize=(16, 12)) 
            plt.scatter(x_data, y_data, label="Data")
            plt.plot(x_fit, y_fit, label="Power Series Fit", color= "red")
            plt.xticks(fontsize=15) 
            plt.yticks(fontsize=15)
            plt.xlabel("Temperature",fontsize=17)
            plt.ylabel("Energy",fontsize=17)
            plt.title("Temperature vs Energy",fontsize=22)
            # Display the equation
            equation = f"y = {k:.2f}*x^{a:.2f}"
            print("Equation:", equation)
            
            
            text_x = 8  # x-coordinate
            text_y = 16  # y-coordinate
            
            plt.text(text_x, text_y, equation, fontsize=19, color='red',
                     bbox={'facecolor': 'white', 'alpha': 0.7, 'edgecolor': 'gray'})
            plt.show()

            return jsonify({"k": k, "a": a})

    def start(self):
        self.run(port=8080, host="0.0.0.0")
