import numpy as np
from bolero.representation import DMPBehavior as DMP


class DMPBehavior(DMP):
    """Ignores the velocities and accelerations from the DMP."""
    def __init__(self, *args, **kwargs):
        super(DMPBehavior, self).__init__(*args, **kwargs)

    def init(self, n_inputs, n_outputs):
        super(DMPBehavior, self).init(n_inputs * 3, n_outputs * 3)
        self.n_joints = n_inputs
        self.inputs = np.empty(3 * self.n_joints)
        self.outputs = np.empty(3 * self.n_joints)
        self.initialized = False

    def set_meta_parameters(self, keys, meta_parameters):
        super(DMPBehavior, self).set_meta_parameters(keys, meta_parameters)

    def set_inputs(self, inputs):
        self.y[:] = inputs[:]
        if not self.initialized:
            self.x0 = self.y.copy()
            self.yd[:] = 0.0
            self.ydd[:] = 0.0
            self.initialized = True

        self.inputs[:self.n_joints] = self.y
        self.inputs[self.n_joints:2 * self.n_joints] = self.yd
        self.inputs[2 * self.n_joints:] = self.ydd

        super(DMPBehavior, self).set_inputs(self.inputs)

    def get_outputs(self, outputs):
        super(DMPBehavior, self).get_outputs(self.outputs)
        outputs[:] = self.outputs[:self.n_joints]

    def set_meta_parameters(self, keys, meta_parameters):
        super(DMPBehavior, self).set_meta_parameters(keys, meta_parameters)
