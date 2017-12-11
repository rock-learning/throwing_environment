import numpy as np
from bolero.wrapper import CppBLLoader
from bolero.controller import Controller
from bolero.behavior_search import BlackBoxSearch
from bolero.optimizer import CMAESOptimizer
from dmp_behavior import DMPBehavior
from first_feedback import FirstFeedback


if __name__ == "__main__":
    environment_name = "throwing_environment"
    bll = CppBLLoader()
    bll.load_library(environment_name)
    env = bll.acquire_contextual_environment(environment_name)
    env = FirstFeedback(env, random_state=0)
    env.request_context(np.array([1.5, 1.0]))

    beh = DMPBehavior(dt=0.01, execution_time=0.5, n_features=5)

    opt = CMAESOptimizer(variance=200.0 ** 2, active=True, random_state=0)

    bs = BlackBoxSearch(beh, opt)

    ctrl = Controller(environment=env, behavior_search=bs, n_episodes=200,
                      verbose=2)
    meta_params = [np.array([0.0, -0.8, -0.7]), np.array([0.5, 0.5, 0.5]),
                   np.array([0.0, 0.5, 0.5])]
    print(ctrl.learn(["x0", "g", "gd"], meta_params))
