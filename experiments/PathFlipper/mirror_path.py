import json
import os
import shutil
field_width = 8.069
field_length = 16.541
def flip_angle_side(angle):
    return (-angle) % 360
def flip_x(obj):
    if obj:
        obj["y"] = field_width - obj["y"]
    return obj
dir_path = os.path.join(os.getcwd(),'paths')
all_paths = os.listdir(dir_path )
for auto_path in all_paths:
    if auto_path == '.DS_Store':
        continue
    path_ofauto = os.path.join(dir_path,auto_path)
    print(path_ofauto)
    with open(path_ofauto) as f:
        data = json.load(f)
    waypoints = []
    for point in data['waypoints']:
        point['anchor'] = flip_x(point.get("anchor"))
        point['prevControl'] = flip_x(point.get("prevControl"))
        point['nextControl'] = flip_x(point.get("nextControl"))
        if point['linkedName'] != None:
            point['linkedName'] = "mirrored_"+point['linkedName']
        waypoints.append(point)
    rotations_targets = []
    for target in data['rotationTargets']:
        target['rotationDegrees'] = flip_angle_side(target['rotationDegrees'])
        rotations_targets.append(target)
    data['idealStartingState']['rotation'] = flip_angle_side(data['idealStartingState']['rotation'])  
    data['goalEndState']['rotation'] = flip_angle_side(data['goalEndState']['rotation'])  
    data["folder"] =  "mirrored_"+ data["folder"]
    with open(os.path.join(dir_path,"mirrored_"+auto_path), "w") as f:
        json.dump(data, f, indent=2)