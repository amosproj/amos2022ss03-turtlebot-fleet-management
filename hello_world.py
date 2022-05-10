from time import sleep
from flask import Flask

app= Flask(__name__)
@app.route("/")
def index():
    return "Hello from Flask to  Amos 03 Team Tutlebot Fleet Management !!!"

app.run(host="0.0.0.0")
print("Hello World!!!")
sleep(60)
