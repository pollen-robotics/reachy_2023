def identify_zuuu_model():
    import yaml
    import os
    config_file = os.path.expanduser('~/.reachy.yaml')
    with open(config_file) as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        return config['zuuu_version']