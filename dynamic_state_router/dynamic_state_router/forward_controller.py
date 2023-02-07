from typing import List
import yaml


class ForwardController:
    def __init__(self, name, joints) -> None:
        self.name = name
        self.interface = self._extract_interface(name)

        self.joints = joints
        self.joint2id = {j: i for i, j in enumerate(joints)}
        self.id2joint = {i: j for i, j in enumerate(joints)}

    def joints(self) -> List[str]:
        return self.joints

    def __repr__(self):
        return f'<{self.name}>'

    def _extract_interface(self, name):
        prefix = 'forward_'
        suffix = '_controller'

        i = name.find(prefix) + len(prefix)
        j = name.find(suffix)

        return name[i:j]


class ForwardControllersPool:
    def __init__(self, fcs: List[ForwardController]):
        self.fcs = {
            fc.name: fc
            for fc in fcs
        }

        self.corresponding = {}
        for interface in set(fc.interface for fc in fcs):
            self.corresponding[interface] = {}

            ifcs = self.get_controllers_for_interface(interface) 
            for fc in ifcs:
                for j in fc.joints:
                    self.corresponding[interface][j] = fc

    def get_controllers_for_interface(self, interface):
        return [
            fc
            for fc in self.fcs.values()
            if fc.interface == interface
        ]   

    def get_corresponding_controller(self, joint, interface):
        if interface in ('p_gain', 'i_gain', 'd_gain'):
            interface = 'pid'

        return self.corresponding[interface][joint]

    def all(self):
        return self.fcs.values()

    def __getitem__(self, key):
        return self.fcs[key]

    @classmethod
    def parse(cls, logger, controllers_file):
        fcs = []

        with open(controllers_file, 'r') as f:
            logger.debug(f'Parsing {controllers_file} controllers file.')
            config = yaml.safe_load(f)

            controller_config = config['controller_manager']['ros__parameters']
            forward_controllers = []
            for k, v in controller_config.items():
                try:
                    if v['type'].endswith('ForwardCommandController') or v['type'].endswith('PIDCommandController'):
                        forward_controllers.append(k)
                except (KeyError, TypeError):
                    pass

            for c in forward_controllers:
                fc = ForwardController(
                    name=c,
                    joints=config[c]['ros__parameters']['joints'],
                )
            
                fcs.append(fc)

        return cls(fcs=fcs)