from flask import Flask, render_template

app = Flask(__name__)

@app.route('/')
def hello_world():
    return 'Hello, World! This is a simple Flask app.'

@app.route('/about')
def about():
    return 'This is the About page of the Flask example.'

if __name__ == '__main__':
    app.run(debug=True, port=5000)