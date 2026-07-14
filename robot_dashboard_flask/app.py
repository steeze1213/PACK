from flask import Flask, render_template, jsonify

app = Flask(__name__)


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/health")
def health():
    return jsonify({"status": "ok", "service": "robot_dashboard"})


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False)
