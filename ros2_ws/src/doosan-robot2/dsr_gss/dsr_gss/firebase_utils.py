import os
import time
from datetime import datetime
import firebase_admin
from firebase_admin import credentials, db

key_path = os.path.expanduser("~/ros2_ws/keys/firebaseServiceAccountKey.json")
cred = credentials.Certificate(key_path)
firebase_admin.initialize_app(
    cred, {"databaseURL": "https://ros2-monitor-default-rtdb.firebaseio.com"}
)

# # Get a Database Reference
# ref = db.reference("users")  # Reference to the 'users' node

# # Write Data
# # Overwrite data at 'users/user1'.
# ref.child("user1").set({"name": "Alice", "email": "alice@example.com"})

# # Update specific fields in 'users/user2'
# ref.child("user2").update({"name": "Bob"})

# # Read Data
# data = ref.child("user1").get()
# print(data)

# # Add New Children
# new_user_ref = ref.push()
# new_user_ref.set({"name": "Charlie", "email": "charlie@example.com"})

# # Delete Data
# ref.child("user1").delete()


def get_firebase_db_reference():
    """Get a reference to a specific node in the Firebase Realtime Database."""
    return db.reference("dsr_gss")


def _add_log(level, message):
    millis_timestamp = int(time.time() * 1000)
    seconds_timestamp = millis_timestamp / 1000
    dt_object = datetime.fromtimestamp(seconds_timestamp)
    formatted_time = dt_object.strftime("%Y-%m-%d %H:%M:%S")

    print(f"[{formatted_time}][{level.upper()}] {message}")

    get_firebase_db_reference().child("logs").push(
        {
            "timestamp": int(millis_timestamp),
            "level": level.upper(),
            "message": message,
        }
    )


def debug(message):
    _add_log("INFO", message)


def info(message):
    _add_log("INFO", message)


def warning(message):
    _add_log("INFO", message)


def error(message):
    _add_log("INFO", message)


def increase_planted_count():
    planted_count = get_firebase_db_reference().child("planted_cnt").get()
    get_firebase_db_reference().child("planted_cnt").set(planted_count + 1)
