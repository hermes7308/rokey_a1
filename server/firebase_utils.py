import firebase_admin
from firebase_admin import credentials, db

cred = credentials.Certificate('./keys/firebaseServiceAccountKey.json')
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
