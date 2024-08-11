from scipy.optimize import curve_fit
import numpy as np
from matplotlib import use as mpluse
import matplotlib.pyplot as plt
from io import BytesIO

from flask import Flask, render_template, jsonify, request
from os.path import join, dirname
import base64
import cv2
import json

PNG_START = "data:image/png;base64,"
save_path = join(dirname(__file__), "save.json")

class Server(Flask):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._define_routes()

        self.preview = None
        self.show_preview = False
        
        self.radius = None
        self.x_offset = None
        
        # Calibration parameters
        self.a = self.k = None
        self.m = self.c = None

    def _define_routes(self):
        @self.route("/")
        def index():
            with open(save_path, "r") as f:
                data = json.load(f)
                dist = data["dist"]
                angle = data["angle"]
                f.close()
            return render_template("index.html", a=dist["a"], k=dist["k"], m=angle["m"], c=angle["c"])
            
        @self.route("/radius", methods=["POST"])
        def radius():
            return jsonify({"radius": self.radius})
 
        @self.route("/xOffset", methods=["POST"])
        def x_offset():
            return jsonify({"xOffset": self.x_offset})

        @self.route("/hidePreview", methods=["POST"])
        def hide_preview():
            self.show_preview = False
            return jsonify({})
        
        @self.route("/preview", methods=["POST"])
        def preview():
            if not self.show_preview:
                self.show_preview = True
                
            if self.preview is None:
                return jsonify({"preview": None})
            
            _, img_arr = cv2.imencode(".png", self.preview)
            img_bytes = img_arr.tobytes()
            image = PNG_START + base64.b64encode(img_bytes).decode("utf-8")
            
            return jsonify({"preview": image})
         
        @self.route("/fitAngle", methods=["POST"])
        def fit_angle():
            data = json.loads(request.data)

            def func(x, m, c):
                return m * x + c
        
            self.x_data = data["x"]
            self.y_data = data["trueX"]

            self.initial_guess = [1.0, 0.0]
            params, covariance = curve_fit(func, self.x_data, self.y_data, p0=self.initial_guess)

            self.m, self.c = params
            
            with open(save_path, "r") as f:
                data = json.load(f)
                f.close()
            data["angle"] = {"m": self.m, "c": self.c}
            with open(save_path, "w") as f:
                json.dump(data, f, sort_keys=True, indent=4)
                f.close()

            x_fit = np.linspace(min(self.x_data), max(self.x_data), 137)
            y_fit = func(x_fit, self.m, self.c)

            plt.figure(figsize=(16, 12)) 
            plt.scatter(self.x_data, self.y_data, label="Data")
            plt.plot(x_fit, y_fit, label="Linear Fit", color= "red")
            plt.xticks(fontsize=15) 
            plt.yticks(fontsize=15)
            plt.xlabel("Camera X",fontsize=17)
            plt.ylabel("True X",fontsize=17)
            plt.title("Camera X vs True X",fontsize=22)

            equation = f"y = {self.m:.2f}x + {self.c:.2f}"
            
            plt.text(50, 16, equation, fontsize=19, color='red',
                bbox={'facecolor': 'white', 'alpha': 0.7, 'edgecolor': 'gray'})
            img = BytesIO()
            plt.savefig(img, format="png")
            img.seek(0)
            img_base64 = PNG_START + base64.b64encode(img.read()).decode("utf-8")

            return jsonify({"m": self.m, "c": self.c, "graph": img_base64})

        @self.route("/fitDist", methods=["POST"])
        def fit_dist():
            data = json.loads(request.data)

            self.x_data = data["radii"]
            self.y_data = data["distance"]

            def func(x, k, a):
                return k * pow(x, a)
        
            self.initial_guess = [2000.0, -1.0]
            params, covariance = curve_fit(func, self.x_data, self.y_data, p0=self.initial_guess)

            self.k, self.a = params
            
            with open(save_path, "r") as f:
                data = json.load(f)
                f.close()
            data["dist"] = {"k": self.k, "a": self.a}
            with open(save_path, "w") as f:
                json.dump(data, f, sort_keys=True, indent=4)
                f.close()

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

        @self.route("/calibrateColors", methods=["POST"])
        def calibrate_colors():
            colors = json.loads(request.data)
            
            lower = [255, 255, 255]
            upper = [0, 0, 0]
            for color in colors:
                color = [int(x) for x in color[1:-1].split(", ")]
                for i in range(3):
                    if color[i] < lower[i]:
                        lower[i] = color[i]
                    if color[i] > upper[i]:
                        upper[i] = color[i]
                        
            # Convert from RGB to BGR
            lower = lower[::-1]
            upper = upper[::-1]
            
            # Save to save.json
            with open(save_path, "r") as f:
                data = json.load(f)
                f.close()
            data["color"] = {"lower": lower, "upper": upper}
            with open(save_path, "w") as f:
                json.dump(data, f, sort_keys=True, indent=4)
                f.close()
                        
            return jsonify({})

    def start(self):
        # Agg is a non-interactive backend.
        # Prevents the error when initialising a MPL figure.
        mpluse("agg")
        self.run(port=8080, host="0.0.0.0")
