# ---------------------------------------------------------------------------- #
#                                    IMPORTS                                   #
# ---------------------------------------------------------------------------- #

# ----------------------- Standard Python Util Imports ----------------------- #
import time
from typing import Optional, Tuple, Any, Dict
from abc import ABC
import logging
from pprint import pformat


# --------------------------- Sim and Game Imports --------------------------- #
from carla_client.configuration import Configuration as CarlaConfig
from carla_client.carla_runner import CarlaRunner
import pygame

# ------------------------------- Agent Imports ------------------------------ #
from agent_module.configuration import Configuration as AgentConfig
from agent_module.pure_pursuit_agent import PurePursuitAgent
from agent_module.agent import Agent

# -------------------------------- RL Imports -------------------------------- #
import gym
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.base_class import BaseAlgorithm


class ROAREnv(gym.Env, ABC):
    def __init__(self, params: Dict[str, Any]):
        """

        carla_config: CarlaConfig,
        agent_config: AgentConfig,
        npc_agent_class, num_frames_per_step: int = 1,
        use_manual_control: bool = False
        
        Args:
        params:
        """
        carla_config: CarlaConfig = params["carla_config"]
        agent_config: AgentConfig = params["agent_config"]
        ego_agent_class = params.get("ego_agent_class", Agent)
        npc_agent_class = params.get("npc_agent_class", PurePursuitAgent)

        num_frames_per_step: int = params.get("num_frames_per_step", 1)
        # use_manual_control: bool = params.get("use_manual_control", False)
        self.max_collision_allowed: int = params.get("max_collision", 0)
        self.logger = logging.getLogger("ROAR Gym")
        self.agent_config = agent_config
        self.EgoAgentClass = ego_agent_class
        self.npc_agent_class = npc_agent_class
        self.carla_config = carla_config
        self.carla_runner = CarlaRunner(carla_settings=self.carla_config,
                                        agent_settings=self.agent_config,
                                        npc_agent_class=self.npc_agent_class)
        self.num_frames_per_step = num_frames_per_step
        self.agent: Optional[Agent] = None
        self.clock: Optional[pygame.time.Clock] = None

        self.action_space = None  # overwrite in higher classes
        self.observation_space = None  # overwrite in higher classes

    def step(self, action: Any,update_queue=True) -> Tuple[float, bool]:
        """
        This provides an example implementation of step, intended to be overwritten

        Args:
            action: Any

        Returns:
            Tuple of Observation, reward, is done, other information
        """
        agent_control=self.agent.kwargs.get("control")
        carla_control = self.carla_runner.carla_bridge.convert_control_from_agent_to_source(agent_control)
        self.carla_runner.world.player.apply_control(carla_control)

        self.clock.tick()
        should_continue, carla_control = self.carla_runner.controller.parse_events(client=self.carla_runner.client,
                                                                                   world=self.carla_runner.world,
                                                                                   clock=self.clock)
        self.carla_runner.world.tick(self.clock)
        self.carla_runner.convert_data()

        self.agent.run_step(vehicle=self.carla_runner.vehicle_state,update_queue=update_queue)
        return self.get_reward(), self._terminal()

    def reset(self) -> Any:
        self.carla_runner.on_finish()
        self.carla_runner = CarlaRunner(agent_settings=self.agent_config,
                                        carla_settings=self.carla_config,
                                        npc_agent_class=self.npc_agent_class)
        vehicle = self.carla_runner.set_carla_world()

        if self.agent:
            self.agent.reset(vehicle=vehicle)
        else:
            self.agent = self.EgoAgentClass(vehicle=vehicle, agent_settings=self.agent_config)
        self.clock: Optional[pygame.time.Clock] = None
        self._start_game()
        return self.agent

    def render(self, mode='ego'):
        self.carla_runner.world.render(display=self.carla_runner.display)
        pygame.display.flip()

    def _start_game(self):
        try:
            self.logger.debug("Initiating game")
            self.agent.start_module_threads()
            self.clock = pygame.time.Clock()
            self.start_simulation_time = self.carla_runner.world.hud.simulation_time
            self.start_vehicle_position = self.agent.vehicle.transform.location.to_array()
        except Exception as e:
            self.logger.error(e)

    def get_reward(self) -> float:
        """
        Intended to be overwritten
        Returns:

        """
        return -1

    def _terminal(self) -> bool:
        if self.carla_runner.get_num_collision() > self.max_collision_allowed:
            return True
        return self.agent.is_done  # TODO temporary, needs to be changed

    def _get_info(self) -> dict:
        return dict()

    def _get_obs(self) -> Any:
        return self.agent


class LoggingCallback(BaseCallback):
    def __init__(self, model: BaseAlgorithm, verbose=0):
        super().__init__(verbose)
        self.init_callback(model=model)

    def _on_step(self) -> bool:
        curr_step = self.locals.get("step")
        info = {
            # "num_collected_steps": self.locals.get("num_collected_steps"),
            # "reward": self.locals.get("reward"),
            # "episode_rewards": self.locals.get("episode_rewards"),
            # "action": self.locals.get("action"),
            # "iteration": self.locals.get("iteration"),
            # "rewards": self.locals.get("rewards"),
            "infos": self.locals.get("infos")
        }

        msg = f"{pformat(info)}"
        self.logger.log(msg)
        return True
