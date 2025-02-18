from flask import Flask, render_template, request, redirect
import os
import json
app = Flask(__name__, template_folder =os.path.abspath("./"))

#Render the app
@app.route ("/")
def main():
    return render_template("index.html")

#API to get data from the racket, returns json containing all the data from the sensors
@app.route ("/getData")
def getData():
    return {
        "accelerometer":
            {"x":[1.004,1.004,0.969,0.973,1.0,0.961,0.973,1.0,0.969,0.973,1.012,0.996,0.988,1.012,0.996,0.965,1.004,0.988,0.969,1.039,0.984,1.008,0.996,0.988,0.965,0.992,0.992,0.973,1.031,0.984,0.984,1.0,1.031,0.965,1.012,1.008,0.965,1.004,1.008,0.984,1.031,0.977,0.98,1.02,0.98,0.977,1.023,0.992,1.004,0.996,1.0,0.965,0.973,1.008,0.965,0.957,1.012,0.969,1.0,1.016,0.988,0.992,1.0,0.992,0.969,1.0,0.984,1.031,0.996,0.973,0.961,0.988,0.984,0.965,0.992,0.996,0.969,0.996,1.027,0.977,0.996,1.047,0.973,0.988,1.02,1.004,1.023,0.996,0.973,1.031,0.984,0.973,1.027,0.988,0.98,1.027,0.977,1.023,0.969,0.98,0.984,0.973,1.004,0.973,0.973,1.016,0.996,0.961,1.004,0.98,0.973,1.027,0.992,0.988,1.023,1.004,1.031,0.992,0.98,0.973,1.004,1.012,0.965,1.004,1.031,0.996,0.996,1.023,0.957,1.004,1.023,1.0,1.012,0.988,0.977,1.023,0.988,0.977,1.02,0.984,0.977,1.031,0.996,0.98,1.039,0.973,1.016,0.992,1.0,0.957,0.988,1.004,0.965,1.008,1.008,0.969,1.008,1.016,1.016,0.965,1.02,0.984,1.0,0.984,0.984,0.973,1.0,0.98,0.969,0.988,1.02,0.969,1.004,1.035,0.973,0.996,1.031,0.961,1.004,1.027,1.0,1.012,0.988,0.977,1.016,0.977,0.977,1.027,0.98,0.969,1.008,0.977,0.98,1.039,0.988,0.98,1.035,0.973,1.02,0.988,0.996,0.957,1.012,1.008,0.961,1.004,1.0,0.988,1.027,0.973,0.984,0.988,0.98,0.965,0.992,0.992,0.973,1.035,1.008,1.035,0.961,1.0,0.965,0.969,0.992,1.039,1.012,1.0,0.953,0.965,1.004,0.977,0.965,1.02,0.992,0.965,1.031,0.984,0.98,1.027,0.98,0.98,1.02,0.98,0.965,1.008,0.98,1.039,1.004,0.992,0.969,1.0,1.008,0.973,0.992,1.023,0.957,0.992,1.012,1.031,1.008,0.992,1.027,0.984,0.988,1.02,1.031,0.992,0.98,1.031,0.988,1.047,0.996,1.004,0.973,0.973,1.012,0.977,0.973,1.008,0.988,0.969,1.016,0.988,0.957,1.031,0.98,0.977,1.023,0.984,0.938,1.023,0.984,1.027,1.004,0.988,0.98,1.004,1.0,0.969,0.996,1.004,0.957,1.008,1.012,1.023,1.008,0.973,0.977,0.984,0.988,0.98,1.035,0.992,0.984,1.027,0.992,1.051,0.977,0.988,1.031,1.0,0.977,0.969,0.969,1.012,0.961,0.969,1.027,0.961,0.977,1.023,0.98,0.98,1.012,0.988,0.973,0.957,0.977,0.973,1.039,1.008,1.039,1.004,1.008,0.969,0.996,1.008,0.969,0.996,1.027,0.988,1.008,0.977,0.973,0.984,0.988,1.039,0.977,0.984,0.984,1.039,0.984,1.027,0.98,0.992,1.035,1.051,0.992,0.965,0.965,1.0,0.973,0.961,1.02,0.973,0.969,1.023,0.988,0.98,0.988,0.988,0.988,1.016,0.988,0.977,1.016,0.973,1.055,0.996,0.996,0.965,0.992,0.984,0.965,0.984,0.996,0.996,1.008,1.023]
            ,"y":[0.09,-0.043,0.09,-0.012,-0.016,0.082,-0.055,0.012,-0.102,-0.059,0.043,-0.109,-0.02,0.043,-0.09,-0.102,0.055,-0.059,-0.086,0.094,0.113,0.012,0.035,-0.066,-0.156,0.059,-0.031,-0.133,0.102,-0.035,-0.004,0.098,-0.012,-0.148,0.102,-0.008,-0.129,0.145,0.004,0.027,0.07,-0.051,-0.094,0.082,-0.012,-0.09,0.109,0.008,0.004,0.078,-0.039,0.113,-0.027,-0.02,0.09,-0.047,-0.004,-0.051,0.121,0.023,-0.121,-0.035,0.039,-0.094,-0.086,0.051,0.082,0.102,0.0,-0.09,-0.023,0.023,-0.082,-0.152,0.008,-0.051,-0.141,0.094,-0.031,-0.09,0.109,-0.016,-0.133,0.055,0.008,0.148,0.078,-0.031,-0.086,0.082,-0.043,-0.078,0.074,-0.008,-0.094,0.105,0.012,0.09,0.102,-0.02,0.008,-0.043,0.0,-0.137,-0.039,0.012,-0.125,-0.086,0.035,-0.082,-0.078,0.066,-0.063,-0.051,0.078,0.109,0.117,0.016,-0.059,-0.152,0.074,-0.027,-0.133,0.086,-0.051,-0.031,0.086,-0.02,-0.152,0.063,0.004,0.141,0.063,-0.059,-0.063,0.074,-0.051,-0.094,0.051,-0.031,-0.109,0.098,-0.004,-0.082,0.117,0.023,0.016,0.09,-0.023,-0.148,0.043,-0.004,-0.137,0.141,-0.012,-0.121,0.148,0.031,-0.004,-0.07,0.047,0.141,0.055,-0.008,0.121,-0.137,-0.008,0.082,-0.129,0.047,-0.043,-0.113,0.063,-0.063,-0.094,0.066,-0.035,-0.156,0.059,0.035,0.141,0.043,-0.07,-0.063,0.063,-0.09,-0.074,0.059,-0.063,-0.102,0.066,-0.035,-0.09,0.109,-0.004,-0.066,0.098,0.02,0.082,0.078,-0.031,-0.156,0.066,-0.02,-0.133,0.141,0.016,0.121,0.07,-0.059,0.074,-0.066,-0.031,0.121,-0.063,-0.004,-0.09,0.117,0.004,-0.074,-0.137,0.043,-0.031,-0.129,0.078,0.082,0.043,0.016,-0.113,-0.078,-0.008,0.039,-0.086,0.059,-0.02,-0.082,0.066,-0.07,-0.031,0.078,-0.047,-0.09,0.051,-0.016,-0.102,0.098,0.051,0.102,0.094,-0.039,-0.109,0.109,-0.027,-0.105,0.098,-0.004,-0.141,0.039,0.02,0.133,0.094,0.004,-0.078,-0.094,-0.027,-0.035,0.094,-0.004,-0.094,0.047,0.027,0.102,0.121,-0.004,0.082,-0.063,0.004,-0.082,-0.07,0.031,-0.09,-0.055,0.043,-0.098,-0.043,0.059,-0.066,-0.094,0.012,-0.035,-0.102,0.098,0.039,0.102,0.086,-0.047,-0.082,0.09,-0.039,-0.125,0.086,-0.008,-0.145,0.059,0.008,0.125,0.082,0.008,-0.082,-0.09,-0.027,-0.035,0.098,-0.008,-0.094,0.066,0.023,-0.078,-0.113,0.043,0.082,0.137,0.027,-0.105,-0.07,0.023,-0.004,-0.066,0.039,-0.105,-0.031,0.059,-0.078,-0.086,0.055,-0.051,-0.105,0.051,-0.016,-0.098,0.113,0.02,0.098,0.086,-0.047,-0.074,0.102,-0.008,-0.129,0.094,0.0,0.148,0.063,-0.059,0.145,-0.082,0.012,-0.086,-0.105,-0.012,-0.039,0.098,0.023,-0.066,-0.098,0.043,0.105,0.02,-0.004,0.086,-0.066,-0.008,0.063,-0.074,0.035,-0.066,-0.078,0.063,-0.051,-0.066,0.063,-0.059,-0.023,0.082,-0.039,-0.094,0.063,0.086,0.125,0.066,-0.035,-0.152,0.109,-0.012,-0.129,0.121,-0.016,-0.055,0.125,0.008]
            ,"z":[-0.125,-0.125,-0.121,-0.137,-0.121,-0.121,-0.109,-0.133,-0.141,-0.125,-0.133,-0.148,-0.125,-0.129,-0.152,-0.117,-0.121,-0.141,-0.117,-0.117,-0.148,-0.141,-0.141,-0.125,-0.113,-0.129,-0.125,-0.148,-0.145,-0.125,-0.137,-0.125,-0.129,-0.152,-0.129,-0.133,-0.164,-0.121,-0.121,-0.121,-0.117,-0.152,-0.117,-0.125,-0.137,-0.125,-0.109,-0.137,-0.133,-0.133,-0.125,-0.117,-0.117,-0.121,-0.121,-0.113,-0.125,-0.133,-0.125,-0.121,-0.156,-0.137,-0.125,-0.156,-0.129,-0.121,-0.148,-0.121,-0.145,-0.125,-0.133,-0.137,-0.121,-0.148,-0.129,-0.121,-0.148,-0.141,-0.133,-0.145,-0.133,-0.113,-0.148,-0.129,-0.129,-0.121,-0.121,-0.137,-0.113,-0.121,-0.152,-0.125,-0.125,-0.141,-0.125,-0.121,-0.145,-0.125,-0.105,-0.117,-0.125,-0.117,-0.125,-0.156,-0.121,-0.133,-0.152,-0.113,-0.129,-0.152,-0.121,-0.121,-0.145,-0.125,-0.125,-0.145,-0.121,-0.137,-0.125,-0.152,-0.137,-0.137,-0.148,-0.125,-0.133,-0.141,-0.129,-0.121,-0.156,-0.121,-0.133,-0.117,-0.125,-0.145,-0.121,-0.129,-0.156,-0.109,-0.125,-0.148,-0.125,-0.129,-0.145,-0.121,-0.121,-0.137,-0.125,-0.133,-0.148,-0.113,-0.125,-0.137,-0.109,-0.125,-0.121,-0.121,-0.121,-0.129,-0.129,-0.117,-0.125,-0.152,-0.125,-0.137,-0.117,-0.137,-0.129,-0.125,-0.137,-0.137,-0.129,-0.141,-0.133,-0.129,-0.141,-0.137,-0.133,-0.152,-0.133,-0.125,-0.125,-0.125,-0.148,-0.117,-0.125,-0.156,-0.125,-0.125,-0.145,-0.121,-0.125,-0.145,-0.121,-0.117,-0.137,-0.125,-0.117,-0.141,-0.125,-0.129,-0.133,-0.117,-0.125,-0.129,-0.152,-0.129,-0.125,-0.156,-0.113,-0.156,-0.117,-0.137,-0.148,-0.098,-0.129,-0.141,-0.121,-0.121,-0.133,-0.129,-0.148,-0.141,-0.129,-0.152,-0.133,-0.125,-0.133,-0.117,-0.141,-0.125,-0.125,-0.117,-0.117,-0.117,-0.148,-0.129,-0.117,-0.16,-0.125,-0.125,-0.145,-0.125,-0.125,-0.148,-0.113,-0.121,-0.145,-0.125,-0.133,-0.129,-0.148,-0.125,-0.125,-0.141,-0.129,-0.125,-0.156,-0.125,-0.129,-0.145,-0.125,-0.145,-0.133,-0.145,-0.145,-0.109,-0.125,-0.141,-0.117,-0.133,-0.141,-0.133,-0.129,-0.137,-0.125,-0.121,-0.133,-0.137,-0.121,-0.125,-0.148,-0.125,-0.125,-0.156,-0.125,-0.125,-0.156,-0.113,-0.137,-0.145,-0.105,-0.113,-0.141,-0.117,-0.137,-0.121,-0.121,-0.133,-0.129,-0.145,-0.141,-0.129,-0.156,-0.125,-0.121,-0.156,-0.121,-0.117,-0.121,-0.137,-0.145,-0.121,-0.125,-0.141,-0.121,-0.125,-0.145,-0.133,-0.145,-0.137,-0.125,-0.113,-0.137,-0.145,-0.121,-0.129,-0.129,-0.125,-0.125,-0.148,-0.125,-0.113,-0.148,-0.121,-0.125,-0.16,-0.121,-0.129,-0.141,-0.125,-0.113,-0.117,-0.121,-0.125,-0.129,-0.133,-0.133,-0.129,-0.145,-0.133,-0.129,-0.152,-0.125,-0.148,-0.129,-0.125,-0.133,-0.125,-0.141,-0.141,-0.125,-0.117,-0.145,-0.137,-0.137,-0.137,-0.121,-0.117,-0.117,-0.121,-0.129,-0.137,-0.129,-0.117,-0.125,-0.156,-0.117,-0.125,-0.145,-0.121,-0.133,-0.141,-0.121,-0.125,-0.145,-0.125,-0.121,-0.137,-0.113,-0.133,-0.125,-0.121,-0.137,-0.137,-0.148,-0.137,-0.121,-0.148,-0.133,-0.129]
            },
        "gyro":{},
        "piezo":{}
    }

if __name__ == "__main__":
   app.run(host='0.0.0.0', port=8181, debug=True)