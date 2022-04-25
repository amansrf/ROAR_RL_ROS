# ---------------------------------------------------------------------------- #
#                                    IMPORTS                                   #
# ---------------------------------------------------------------------------- #

# ----------------------- Standard Python Util Imports ----------------------- #
from typing import Tuple
from typing import List, Any
import math
from collections import OrderedDict

# -------------------------- Python Helper Libraries ------------------------- #
import numpy as np
import cv2

# ------------------------------ Vehicle Imports ----------------------------- #
from utilities_module.vehicle_models import Vehicle, VehicleControl

# ------------------------------- Agent Imports ------------------------------ #
from agent_module.agent import Agent

# -------------------------------- RL Imports -------------------------------- #
import gym
from Rgym.spaces import Discrete, Box

# ----------------------------- Wandb Integration ---------------------------- #
import wandb

# ----------------------------- Base Class Import ---------------------------- #
from roar_env.roar_env import ROAREnv

# ----------------------------- ROS Import ---------------------------- #
import rclpy
from roar_env.control_streamer import ControlStreamer
from roar_env.state_streamer import StateStreamer
from roar_env.bev_module import BEVmodule
FRAME_STACK = 4
FRAME_SIZE = {
    "x_res": 84,
    "y_res": 84
}


class ROARppoEnvE2E(ROAREnv):
    def __init__(self, params):
        super().__init__(params)

        low=np.array([-2.5, -5.0, 1.0])
        high=np.array([-0.5, 5.0, 3.0])

        self.action_space = Box(low=low, high=high, dtype=np.float32)
        self.observation_space = Box(-10, 1, shape=(FRAME_STACK,3, FRAME_SIZE["x_res"], FRAME_SIZE["y_res"]), dtype=np.float32)
        
        self.prev_speed = 0
        self.prev_cross_reward = 0
        self.crash_check = False
        self.ep_rewards = 0
        self.frame_reward = 0
        self.highscore = -1000
        self.highest_chkpt = 0
        self.speeds = []
        self.prev_int_counter = 0
        self.steps=0
        self.largest_steps=0
        self.highspeed=0
        self.complete_loop=False
        self.his_checkpoint=[]
        self.his_score=[]
        self.time_to_waypoint_ratio = 0.25
        # self.crash_step=0
        # self.reward_step=0
        # self.reset_by_crash=True
        self.fps=8
        # self.crash_tol=5
        # self.reward_tol=5
        # self.end_check=False
        self.death_line_dis = 5


        rclpy.init()
        self.cntrl_pub_node = ControlStreamer()
        self.state_sub_node = StateStreamer()
        self.brv_sub_node = BEVmodule()
        rclpy.spin_once(self.state_sub_node)
        rclpy.spin_once(self.brv_sub_node)


    def step(self, action: Any) -> Tuple[Any, float, bool, dict]:
        obs = []
        rewards = []
        self.steps+=1
        for i in range(1):

            throttle = 0.6
            steering = action[i*3+1]/5
            braking = 0

            self.agent.kwargs["control"] = VehicleControl(throttle=throttle,
                                                          steering=steering,
                                                          braking=braking)

            # ---------------------------------------------------------------------------- #
            #              TODO: Add Control Publisher Here Using values above             #
            # ---------------------------------------------------------------------------- #

            self.cntrl_pub_node.pub_cntrl(throttle=throttle, steer = steering, brake = float(braking))

            ob, reward, is_done, info = super(ROARppoEnvE2E, self).step(action)
            obs.append(ob)
            rewards.append(reward)
            if is_done:
                break
        self.render()
        self.frame_reward = sum(rewards)
        self.ep_rewards += sum(rewards)
        if is_done:
            self.wandb_logger()
            self.crash_check = False
            self.update_highscore()
        return np.array(obs), self.frame_reward, self._terminal(), self._get_info()

    def _get_info(self) -> dict:
        info_dict = OrderedDict()
        info_dict["Current HIGHSCORE"] = self.highscore
        info_dict["Furthest Checkpoint"] = self.highest_chkpt*self.agent.interval
        info_dict["episode reward"] = self.ep_rewards
        info_dict["checkpoints"] = self.agent.int_counter*self.agent.interval
        info_dict["reward"] = self.frame_reward
        info_dict["largest_steps"] = self.largest_steps
        info_dict["highest_speed"] = self.highspeed
        info_dict["complete_state"]=self.complete_loop
        info_dict["avg10_checkpoints"]=np.average(self.his_checkpoint)
        info_dict["avg10_score"]=np.average(self.his_score)
        # info_dict["throttle"] = action[0]
        # info_dict["steering"] = action[1]
        # info_dict["braking"] = action[2]
        return info_dict

    def update_highscore(self):
        if self.ep_rewards > self.highscore:
            self.highscore = self.ep_rewards
        if self.agent.int_counter > self.highest_chkpt:
            self.highest_chkpt = self.agent.int_counter
        if self.agent.vehicle.get_speed(self.agent.vehicle)>self.highspeed:
            self.highspeed=self.agent.vehicle.get_speed(self.agent.vehicle)
        return

    def _terminal(self) -> bool:
        if not (self.agent.bbox_list[(self.agent.int_counter - self.death_line_dis) % len(self.agent.bbox_list)].has_crossed(self.agent.vehicle.transform))[0]:
            return True
        if self.carla_runner.get_num_collision() > self.max_collision_allowed:
            return True
        elif self.agent.finish_loop:
            self.complete_loop=True
            return True
        else:
            return False

    def get_reward(self) -> float:
        
        # ---------------------------------------------------------------------------------------------------------------------------#
        #              TODO: Need to add subsciber function here (check for crash, IR sensor and Ultrasonic data values)             #
        # -------------------------------------------------------------------------------------------------------------------------- #
        #Create a RL_reward node  which subscribes to the state node 

        reward=-1

        
       #self.state_sub_node = get data for crash and reward and input it in the following steps


        if self.crash_check:
            print("no reward")
            return 0
        

        if self.agent.cross_reward > self.prev_cross_reward:
            reward += (self.agent.cross_reward - self.prev_cross_reward)*self.agent.interval*self.time_to_waypoint_ratio


        if not (self.agent.bbox_list[(self.agent.int_counter - self.death_line_dis) % len(self.agent.bbox_list)].has_crossed(self.agent.vehicle.transform))[0]:
            reward -= 200
            self.crash_check = True
            print("BADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBADBAD")
        
        if self.carla_runner.get_num_collision() > 0:
            reward -= 200
            self.crash_check = True


        # log prev info for next reward computation
        self.prev_speed = Vehicle.get_speed(self.agent.vehicle)
        self.prev_cross_reward = self.agent.cross_reward
        return reward

    def _get_obs(self) -> np.ndarray:
        # ---------------------------------------------------------------------------------------------------------------------------#
        #              TODO: Need to add subsciber function here (BEV)  I THINK!                                                     #
        # -------------------------------------------------------------------------------------------------------------------------- #
        #Create a RL_obs node  which subscribes to the BEV Module (node). RL_env receives the info tfrom the RL_obs node 


        #self.bev_sub_node = get data for occupancy grid and give it to the following

        if mode=='baseline':
            # vehicle_state=self.agent.vehicle.to_array() #12
            # line_location=self.agent.bbox.to_array(vehicle_state[3],vehicle_state[5]) #4
            # v_speed=np.sqrt(np.square(vehicle_state[0])+np.square(vehicle_state[1]))/150
            # v_height=vehicle_state[4]/100
            # v_roll,v_pitch,v_yaw=vehicle_state[[6,7,8]]/180
            # v_throttle,v_steering,v_braking=vehicle_state[[9,10,11]]
            # x_dis,y_dis,xy_dis=line_location[:3]/40
            # l_yaw,vtol_yaw=line_location[3:]
            # data=np.array([v_speed,v_height,v_roll,v_pitch,v_yaw,v_throttle,v_steering,v_braking,x_dis,y_dis,xy_dis,l_yaw,vtol_yaw])
            # l=len(self.agent.bbox_list)
            index_from=(self.agent.int_counter%len(self.agent.bbox_list))
            if index_from+10<=len(self.agent.bbox_list):
                # print(index_from,len(self.agent.bbox_list),index_from+10-len(self.agent.bbox_list))
                next_bbox_list=self.agent.bbox_list[index_from:index_from+10]
            else:
                # print(index_from,len(self.agent.bbox_list),index_from+10-len(self.agent.bbox_list))
                next_bbox_list=self.agent.bbox_list[index_from:]+self.agent.bbox_list[:index_from+10-len(self.agent.bbox_list)]
            assert(len(next_bbox_list)==10)
            map_list = self.agent.occupancy_map.get_map_baseline(transform_list=self.agent.vt_queue,
                                                    view_size=(FRAME_SIZE["x_res"], FRAME_SIZE["y_res"]),
                                                    bbox_list=self.agent.frame_queue,
                                                                 next_bbox_list=next_bbox_list
                                                    )
            # data = cv2.resize(occu_map, (FRAME_SIZE["x_res"], FRAME_SIZE["y_res"]), interpolation=cv2.INTER_AREA)
            #cv2.imshow("Occupancy Grid Map", cv2.resize(np.float32(data), dsize=(500, 500)))

            # data_view=np.sum(data,axis=2)
            cv2.imshow("data", np.hstack(np.hstack(map_list))) # uncomment to show occu map
            cv2.waitKey(1)
            # yaw_angle=self.agent.vehicle.transform.rotation.yaw
            # velocity=self.agent.vehicle.get_speed(self.agent.vehicle)
            # data[0,0,2]=velocity
            # map_input=map_list[:,0]
            # map_input*=255
            # waypoint=np.sum(map_list[:,1:3],axis=1)
            # waypoint*=255
            # data_input=np.zeros_like(map_list)
            # data_input[0,:13]=data
            #print(map_list[:,:-1].shape)
            return map_list[:,:-1]

        else:
            data = self.agent.occupancy_map.get_map(transform=self.agent.vehicle.transform,
                                                    view_size=(FRAME_SIZE["x_res"], FRAME_SIZE["y_res"]),
                                                    arbitrary_locations=self.agent.bbox.get_visualize_locs(),
                                                    arbitrary_point_value=self.agent.bbox.get_value(),
                                                    vehicle_velocity=self.agent.vehicle.velocity,
                                                    # rotate=self.agent.bbox.get_yaw()
                                                    )
            # data = cv2.resize(occu_map, (FRAME_SIZE["x_res"], FRAME_SIZE["y_res"]), interpolation=cv2.INTER_AREA)
            #cv2.imshow("Occupancy Grid Map", cv2.resize(np.float32(data), dsize=(500, 500)))

            # data_view=np.sum(data,axis=2)
            cv2.imshow("data", data) # uncomment to show occu map
            cv2.waitKey(1)
            # yaw_angle=self.agent.vehicle.transform.rotation.yaw
            # velocity=self.agent.vehicle.get_speed(self.agent.vehicle)
            # data[0,0,2]=velocity
            data_input=data.copy()
            data_input[data_input==1]=-10
            return data_input  # height x width x 3 array
    #3location 3 rotation 3velocity 20 waypoline locations 20 wayline rewards

    def reset(self) -> Any:
        if len(self.his_checkpoint)>=10:
            self.his_checkpoint=self.his_checkpoint[-10:]
            self.his_score=self.his_score[-10:]
        if self.agent:
            self.his_checkpoint.append(self.agent.int_counter*self.agent.interval)
            self.his_score.append(self.ep_rewards)
        self.ep_rewards = 0
        if self.steps>self.largest_steps and not self.complete_loop:
            self.largest_steps=self.steps
        elif self.complete_loop and self.agent.finish_loop and self.steps<self.largest_steps:
            self.largest_steps=self.steps
        super(ROARppoEnvE2E, self).reset()
        self.steps=0
        # self.crash_step=0
        # self.reward_step=0
        return self._get_obs()

    def wandb_logger(self):
        wandb.log({
            "Episode reward": self.ep_rewards,
            "Checkpoint reached": self.agent.int_counter*self.agent.interval,
            "largest_steps" : self.largest_steps,
            "highest_speed" : self.highspeed,
            "avg10_checkpoints":np.average(self.his_checkpoint),
            "avg10_score":np.average(self.his_score),
        })
        return