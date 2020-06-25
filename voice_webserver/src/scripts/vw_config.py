import json
import rospkg 
import os.path
import argparse

pth = rospkg.RosPack().get_path("voice_webserver")
parser = argparse.ArgumentParser()
parser.add_argument("--update_hostname", dest="hostname", type=str, default="")
parser.add_argument("--gpu", dest="gpu", type=int)

if __name__ == "__main__":
    args = parser.parse_args()
    json_path = os.path.join(pth, "src/config/default.json")
    data = {}
    with open(json_path, "r") as f:
        data = json.load(f)
    if args.hostname == "": #runtime - load hostname        
        print(data)["config"]["HOSTNAME"]
    else: #set config
        data["config"]["HOSTNAME"] = args.hostname
        data["config"]["GPU"] = bool(args.gpu)
        with open(json_path, "w") as f:
            json.dump(data, f)
        