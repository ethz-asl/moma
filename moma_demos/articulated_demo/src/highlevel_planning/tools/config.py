import yaml
from copy import deepcopy


class ConfigYaml:
    def __init__(self, filepath):
        with open(filepath, "r") as f:
            self._cfg = yaml.load(f)

    def getparam(self, args_list, default_value=None):
        ret = deepcopy(self._cfg)
        try:
            for arg in args_list:
                ret = ret[arg]
            if default_value is not None:
                assert type(default_value) == type(ret)
            return ret
        except ValueError:
            param_name = "/".join(args_list)
            print(
                "".join(("WARNING: Falling back to default for parameter ", param_name))
            )
            return default_value
