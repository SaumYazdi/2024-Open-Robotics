from flask import Flask, render_template, jsonify
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

    def start(self):
        self.run(port=8080, host="0.0.0.0")
