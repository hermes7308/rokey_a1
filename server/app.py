from shlex import join
from flask import Flask, jsonify, Blueprint, request
from flask_cors import CORS
from firebase_utils import get_firebase_db_reference
import time

api_bp = Blueprint("api", __name__, url_prefix="/api")
db_ref = get_firebase_db_reference()


@api_bp.route("/execute_action", methods=["POST"])
def execute_action():
    data = request.get_json()

    previousActionStatus = db_ref.child("control_event/action").child("status").get()

    if previousActionStatus is not None and previousActionStatus != "done":
        return jsonify(
            {
                "status": "failed",
                "message": "Previous action is not finished.",
            }
        )

    db_ref.child("control_event/action").set(
        {
            "actionType": data["actionType"],
            "data": data["data"],
            "status": "ready",
            "timestamp": int(time.time() * 1000),
        }
    )
    return jsonify(
        {
            "status": "success",
            "endpoint": "/api/execute_action",
            "message": f"{data['actionType']} command received and processed.",
        }
    )


@api_bp.route("/get_current_coordinates", methods=["GET"])
def get_coordinates():
    joint_coordinates = db_ref.child("joint_coordinate").get()
    task_coordinates = db_ref.child("task_coordinate").get()

    return jsonify(
        {
            "status": "success",
            "endpoint": "/api/get_current_coordinates",
            "message": "Get Current coordinates.",
            "data": {
                "joint_coordinates": joint_coordinates,
                "task_coordinates": task_coordinates,
            },
        }
    )


app = Flask(__name__)
CORS(app)
app.register_blueprint(api_bp)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
