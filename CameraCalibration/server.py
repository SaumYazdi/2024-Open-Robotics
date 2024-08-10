from scipy.optimize import curve_fit
import numpy as np
import matplotlib.pyplot as plt
from io import BytesIO

from flask import Flask, render_template, jsonify, request
import base64
import cv2
import json

PNG_START = "data:image/png;base64,"

class Server(Flask):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._define_routes()
        
        self.radius = 0
        self.preview = None

        self.a = self.k = None

    def func(self, x, k, a):
        return k * pow(x, a)
        
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
            image = PNG_START + base64.b64encode(img_bytes).decode("utf-8")
            return jsonify({"preview": image})
        
        @self.route("/fit", methods=["POST"])
        def fit():
            data = json.loads(request.data)

            self.x_data = data["radii"]
            self.y_data = data["distance"]

            self.initial_guess = [2000.0, -1.0]
            params, covariance = curve_fit(self.func, self.x_data, self.y_data, p0=self.initial_guess)

            self.k, self.a = params

            x_fit = np.linspace(min(self.x_data), max(self.x_data), 137)
            y_fit = self.func(x_fit, self.k, self.a)

            plt.figure(figsize=(16, 12)) 
            plt.scatter(self.x_data, self.y_data, label="Data")
            plt.plot(x_fit, y_fit, label="Power Series Fit", color= "red")
            plt.xticks(fontsize=15) 
            plt.yticks(fontsize=15)
            plt.xlabel("Radius",fontsize=17)
            plt.ylabel("Distance",fontsize=17)
            plt.title("Radius vs Distance",fontsize=22)

            equation = f"y = {self.k:.2f}x^{self.a:.2f}"
            
            plt.text(50, 16, equation, fontsize=19, color='red',
                bbox={'facecolor': 'white', 'alpha': 0.7, 'edgecolor': 'gray'})
            img = BytesIO()
            plt.savefig(img, format="png")
            img.seek(0)
            img_base64 = PNG_START + base64.b64encode(img.read()).decode("utf-8")

            return jsonify({"k": self.k, "a": self.a, "graph": img_base64})

    def start(self):
        self.run(port=8080, host="0.0.0.0")
