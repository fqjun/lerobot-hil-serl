# Implementation of Piper robot for LeRobot

from dataclasses import dataclass, field
import time
from typing import Any

from lerobot.cameras import CameraConfig, make_cameras_from_configs
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import Robot, RobotConfig

from .piper_sdk_interface import PiperSDKInterface

import queue, threading, time

'''
前置条件：通信接口、SDK或API读写传感器数据、LeRobot环境；

继承Robot和RobotConfig类，

手是：one
第三视角是：two

'''
@RobotConfig.register_subclass("piper")
@dataclass
class PiperConfig(RobotConfig):
    port: str
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "one": OpenCVCameraConfig(
                index_or_path=4,
                fps=30,
                width=640,
                height=480
            ),
            "two": OpenCVCameraConfig(
                index_or_path=0,
                fps=30,
                width=640,
                height=480,
            ),
        }
    )


class Piper(Robot):
    config_class = PiperConfig
    name = "piper"

    def __init__(self, config: PiperConfig):
        super().__init__(config)
        self.sdk = PiperSDKInterface(port=config.port)
        self.cameras = make_cameras_from_configs(config.cameras)

        # 初始位置和安全位置
        self.init_joint_position = [0.0, 0.0, 0.0, 0.0, 0.28, 0.0, 0.0]
        self.safe_disable_position = [0.0, 0.0, 0.0, 0.0, 0.28, 0.0, 0.0]

    # 电机观测
    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"joint_{i}.pos": float for i in range(7)}

    # 图像观测
    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras}

    # 存储每个电机的位置和图像
    @property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}

    @property
    def action_features(self) -> dict:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        # Assume always connected after SDK init
        return True

    def connect(self, calibrate: bool = True) -> None:
        # Already connected in SDK init
        for cam in self.cameras.values():
            cam.connect()
        self.configure()

    def disconnect(self) -> None:
        self.sdk.set_joint_positions(self.safe_disable_position, use_radians=True)
        print("piper disable after 2 seconds")
        time.sleep(2)
        self.sdk.connect(enable=False)
        # self.sdk.disconnect() # 使用connect方法赋值enable为False
        for cam in self.cameras.values():
            cam.disconnect()

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        self.sdk.set_joint_positions(self.init_joint_position, use_radians=True)

    def configure(self) -> None:
        pass

    # 返回机器人传感器值的字典
    def get_observation(self) -> dict[str, Any]:
        # 定义缩放因子
        scale_factors = [57324.840764] * 6 + [1_000_000]
        tmp = self.sdk.get_status()
        for i in range(7):
            tmp[f'joint_{i}.pos'] = tmp[f'joint_{i}.pos'] / scale_factors[i]
        obs_dict = tmp

        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()
        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        # map the action from the leader to joints for the follower
        
        # inference unlocked
        if not isinstance(action, dict):
            raise TypeError("Action must be a dictionary")
        
        # print('action:',action)
        keys = ['joint_0.pos', 'joint_1.pos', 'joint_2.pos',
            'joint_3.pos', 'joint_4.pos', 'joint_5.pos', 'joint_6.pos']
        
        # 是否在采集，遥操作-True，推理-False
        is_teleop = True

        if is_teleop:
            positions = [float(action.get(k, 0.0)) if action.get(k) is not None else 0.0 for k in keys]
            print('*'*80)
            print(positions)
        else:
            # 此处模型生成的action为增量预测
            delta_positions = [float(action.get(k, 0.0)) if action.get(k) is not None else 0.0 for k in keys]
            
            # 获得当前piper的各关节角度，注意单位对齐
            cur_state=self.sdk.get_status()

            # 定义缩放因子
            scale_factors = [57324.840764] * 6 + [1_000_000]
            # 按顺序取值并缩放
            positions = [
                cur_state[f'joint_{i}.pos'] / scale_factors[i] for i in range(7)
            ]
            # 加到position
            # positions = positions + delta_positions
            positions = [p + d for p, d in zip(positions, delta_positions)]
            print('s'*80)
            print('delta_pos:',delta_positions)
            print('cur_sate:',cur_state)
            print('positions:',positions)
            print('t'*80)

        self.sdk.set_joint_positions(positions,use_radians=True)
        return action