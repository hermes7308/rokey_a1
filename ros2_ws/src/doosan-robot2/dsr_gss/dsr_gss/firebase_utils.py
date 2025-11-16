import os
import time
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
    get_firebase_db_reference().child("logs").push(
        {
            "timestamp": int(time.time() * 1000),
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
