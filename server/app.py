from flask import Flask, jsonify, Blueprint, request

api_bp = Blueprint("api", __name__, url_prefix="/api")


@api_bp.route("/")
def api_base():
    return jsonify(
        {
            "status": "success",
            "endpoint": "/api",
            "message": "Welcome to the base API endpoint!",
        }
    )


@api_bp.route("/about")
def api_about():
    return jsonify(
        {
            "status": "success",
            "endpoint": "/api/about",
            "description": "This is the About API page.",
        }
    )

@api_bp.route("/movel", methods=["POST"])
def api_movel():
    data = request.get_json()
    print("MOVEL endpoint called: ", data)
    return jsonify(
        {
            "status": "success",
            "endpoint": "/api/movel",
            "message": "MOVEL command received and processed.",
        }
    )

app = Flask(__name__)
app.register_blueprint(api_bp)

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)
