import numpy as np
from bolero.environment import ContextualEnvironment


class FirstFeedback(ContextualEnvironment):
    """Extract the relevant feedbacks from the environment."""
    def __init__(self, env, random_state=None):
        self.env = env
        if isinstance(random_state, int):
            self.random_state = np.random.RandomState(random_state)
        else:
            self.random_state = random_state

    def init(self):
        self.env.init()

    def reset(self):
        self.env.reset()

    def get_num_inputs(self):
        return self.env.get_num_inputs()

    def get_num_outputs(self):
        return self.env.get_num_outputs()

    def get_outputs(self, values):
        self.env.get_outputs(values)

    def set_inputs(self, values):
        self.env.set_inputs(values)

    def step_action(self):
        self.env.step_action()

    def is_evaluation_done(self):
        return self.env.is_evaluation_done()

    def is_behavior_learning_done(self):
        return self.env.is_behavior_learning_done()

    def request_context(self, context=None):
        if context is None:
            context = self.random_state.uniform([1.0, -1.0], [2.5, 1.0])
        return self.env.request_context(context)

    def get_num_context_dims(self):
        return self.env.get_num_context_dims()

    def get_maximum_feedback(self, context):
        return 0.0

    def get_feedback(self):
        feedbacks = self.env.get_feedback()
        print("Ball hits the ground at %s" % np.round(feedbacks[1:3], 2))
        return feedbacks[0]
