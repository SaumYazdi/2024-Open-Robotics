from flask import Flask, render_template

class Server(Flask):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._define_routes()

    def _define_routes(self):
        @self.route("/")
        def index():
            return render_template("index.html")
            
    def start(self):
        self.run(port=8080, host="0.0.0.0")
