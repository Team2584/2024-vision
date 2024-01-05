from flask import Flask, render_template, request, abort, redirect
from werkzeug.middleware.proxy_fix import ProxyFix
import subprocess
import time

ip_addr = "10.7.2.72"
port = "5801"
delay_time = 6

app = Flask(__name__)

app.wsgi_app = ProxyFix(
        app.wsgi_app, x_for = 1, x_proto = 1, x_host = 1, x_prefix = 1)

def set_mode(mode):
    with open("mode", "w") as f:
        f.write(mode)
    print(mode)

def get_mode():
    with open("mode", "r") as f:
        mode = f.read().strip()
    return mode

@app.route("/")
def default_page():
    print("Default Page")
    set_mode("run")
    return render_template("index.html", mode = get_mode(), ip = ip_addr)

@app.route("/tune")
def tune():
    set_mode("tune")
    return render_template("tune-select.html", mode = get_mode(), ip = ip_addr);

@app.route("/loading/<page>")
def loading_tune(page):
    if (page == "restart"):
        set_mode("restart")
        return render_template("loading.html"), {"Refresh": f"{delay_time}; url=http://{ip_addr}:{port}/"}
    elif (page == "tune-cone"):
        set_mode("tune")
        return render_template("loading.html"), {"Refresh": f"{delay_time}; url=http://{ip_addr}:{port}/tune/cone"}
    elif (page == "tune-cube"):
        set_mode("tune")
        return render_template("loading.html"), {"Refresh": f"{delay_time}; url=http://{ip_addr}:{port}/tune/cube"}
    else:
        set_mode("run")
        return render_template("loading.html"), {"Refresh": f"{delay_time}; url=http://{ip_addr}:{port}/"}

@app.route("/tune/<page>")
def cones_or_cubes(page):
    if not page == "cone" and not page == "cube":
        return redirect("/")

    set_mode("tune")

    filename = page + "-params.txt"

    with open(filename, "r") as f:
        params = f.readlines();

    html_page = page + ".html"
    return render_template(html_page, hue_min = params[0],
                                      hue_max = params[1],
                                      sat_min = params[2],
                                      sat_max = params[3],
                                      val_min = params[4],
                                      val_max = params[5],
                                      mode = get_mode(),
                                      ip = ip_addr)

@app.route("/restart", methods=['POST'])
def restart_code():
    set_mode("restart")
    time.sleep(2)
    set_mode("run")
    return {'did': True}

@app.route("/runvision", methods=['POST'])
def runvision():
    print("Runvision Set")
    set_mode("run")
    return {'did': True}

@app.route("/send-cones", methods=['POST'])
def send_cones():
    data = request.json
    with open("cone-params.txt", "w") as f:
        for item in data.values():
            f.write(item)
            f.write("\n")
    return {'did': True}

@app.route("/send-cubes", methods=['POST'])
def send_cubes():
    data = request.json
    with open("cube-params.txt", "w") as f:
        for item in data.values():
            f.write(item)
            f.write("\n")
    return {'did': True}

@app.errorhandler(404)
def page_not_found(e):
    print("Error 404, Page Not Found")
    print(e)
    if (get_mode() == "tune"):
        return tune()
    else:
        return default_page()

if __name__ == "__main__":
    app.run()
