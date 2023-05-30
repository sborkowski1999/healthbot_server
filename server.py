from flask import Flask, render_template

app = Flask(__name__)

<<<<<<< HEAD
@app.route('/')
def index():
    return render_template('index.html')

if __name__ == '__main__':
    app.run()
=======
@app.route("/")
def script():
    return render_template('index.html')

if __name__ == 'main':
    app.run()
>>>>>>> 3223faba8ae13dfdd1f3b3bf6fe395cc167b2304
