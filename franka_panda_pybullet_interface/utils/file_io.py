import json

import yaml


def load_yaml(filepath):
    with open(filepath, 'r') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    return data


def write_yaml(filepath, data):
    with open(filepath, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)


def load_json(filepath):
    with open(filepath, 'r') as f:
        data = json.load(f)
    return data


def write_json(filepath, data):
    with open(filepath, 'w') as f:
        json.dump(data, f, indent=4)


def load_txt(filepath):
    with open(filepath, 'r') as f:
        data = f.read()
    return data


def write_txt(filepath, data):
    with open(filepath, 'w') as f:
        f.write(data)
