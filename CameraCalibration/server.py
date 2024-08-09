from flask import Flask, render_template, jsonify

class Server(Flask):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._define_routes()
        
        self.distance = 0

    def _define_routes(self):
        @self.route("/")
        def index():
            return render_template("index.html")
            
        @self.route("/distance", methods=["POST"])
        def distance():
            return jsonify({"distance": self.distance})

    def start(self):
        self.run(port=8080, host="0.0.0.0")
