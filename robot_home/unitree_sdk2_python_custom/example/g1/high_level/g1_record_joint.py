#!/usr/bin/env python3
import argparse
import time
import json
import signal
import sys
from typing import List, Optional
from datetime import datetime

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

JOINT_NAMES: List[str] = [
    "LeftHipPitch", "LeftHipRoll", "LeftHipYaw", "LeftKnee", "LeftAnklePitch", "LeftAnkleRoll",
    "RightHipPitch", "RightHipRoll", "RightHipYaw", "RightKnee", "RightAnklePitch", "RightAnkleRoll",
    "WaistYaw", "WaistRoll", "WaistPitch",
    "LeftShoulderPitch", "LeftShoulderRoll", "LeftShoulderYaw", "LeftElbow", "LeftWristRoll", "LeftWristPitch", "LeftWristYaw",
    "RightShoulderPitch", "RightShoulderRoll", "RightShoulderYaw", "RightElbow", "RightWristRoll", "RightWristPitch", "RightWristYaw",
    "Joint29", "Joint30", "Joint31", "Joint32", "Joint33", "Joint34",
]

# Маппинг имен джоинтов в формат 1.json
JOINT_NAME_MAPPING = {
    "LeftHipPitch": "left_hip_pitch_joint",
    "LeftHipRoll": "left_hip_roll_joint",
    "LeftHipYaw": "left_hip_yaw_joint",
    "LeftKnee": "left_knee_joint",
    "LeftAnklePitch": "left_ankle_pitch_joint",
    "LeftAnkleRoll": "left_ankle_roll_joint",
    "RightHipPitch": "right_hip_pitch_joint",
    "RightHipRoll": "right_hip_roll_joint",
    "RightHipYaw": "right_hip_yaw_joint",
    "RightKnee": "right_knee_joint",
    "RightAnklePitch": "right_ankle_pitch_joint",
    "RightAnkleRoll": "right_ankle_roll_joint",
    "WaistYaw": "waist_yaw_joint",
    "WaistRoll": "waist_roll_joint",
    "WaistPitch": "waist_pitch_joint",
    "LeftShoulderPitch": "left_shoulder_pitch_joint",
    "LeftShoulderRoll": "left_shoulder_roll_joint",
    "LeftShoulderYaw": "left_shoulder_yaw_joint",
    "LeftElbow": "left_elbow_joint",
    "LeftWristRoll": "left_wrist_roll_joint",
    "LeftWristPitch": "left_wrist_pitch_joint",
    "LeftWristYaw": "left_wrist_yaw_joint",
    "RightShoulderPitch": "right_shoulder_pitch_joint",
    "RightShoulderRoll": "right_shoulder_roll_joint",
    "RightShoulderYaw": "right_shoulder_yaw_joint",
    "RightElbow": "right_elbow_joint",
    "RightWristRoll": "right_wrist_roll_joint",
    "RightWristPitch": "right_wrist_pitch_joint",
    "RightWristYaw": "right_wrist_yaw_joint",
}


class JointStateRecorder:
    def __init__(self, topic: str):
        self.topic = topic
        self.sub = ChannelSubscriber(topic, LowState_)
        self.frames = []
        self.start_time: Optional[float] = None
        self.last_frame_time: Optional[float] = None
        self.msg_count = 0
        self.running = True

        signal.signal(signal.SIGINT, self.signal_handler)

    def start(self):
        self.sub.Init(self.on_state, 50)
        print(f"Запись состояний джоинтов начата с топика '{self.topic}'")
        print("Нажмите Ctrl+C для остановки и сохранения.")

    def close(self):
        self.sub.Close()

    def on_state(self, msg: LowState_):
        if not self.running:
            return

        if self.start_time is None:
            self.start_time = time.time()
            self.last_frame_time = self.start_time
            print(f"Первое сообщение получено. Начало записи...")

        current_time = time.time()

        # Вычисляем duration с предыдущего фрейма
        duration = current_time - self.last_frame_time if self.last_frame_time else 0
        self.last_frame_time = current_time

        # Формируем frame в формате 1.json
        frame_data = []
        for i, motor in enumerate(msg.motor_state):
            if i < len(JOINT_NAMES):
                joint_name = JOINT_NAMES[i]
                # Используем маппинг если есть, иначе оставляем как есть
                mapped_name = JOINT_NAME_MAPPING.get(joint_name, joint_name.lower())

                # Конвертируем радианы в градусы
                angle_degrees = float(motor.q) * 57.2957795131  # 180/π

                frame_data.append({
                    "name": mapped_name,
                    "angle": angle_degrees
                })

        frame_obj = {
            "frame": frame_data,
            "duration": round(duration, 10)
        }

        self.frames.append(frame_obj)
        self.msg_count += 1

        if self.msg_count % 100 == 0:
            relative_time = current_time - self.start_time
            print(f"Записано фреймов: {self.msg_count}, время: {relative_time:.2f}с")

    def signal_handler(self, sig, frame):
        if not self.running:
            return
        self.running = False
        print("\n\nОстановка записи...")
        self.save_trajectory()
        self.close()
        sys.exit(0)

    def save_trajectory(self):
        if not self.frames:
            print("Нет записанных данных для сохранения")
            return

        import os
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_dir = "/home/unitree/unitree_sdk2_python/example/g1/high_level/data_steer/data_real_states"
        os.makedirs(output_dir, exist_ok=True)
        filename = f"{output_dir}/{timestamp}.json"

        with open(filename, 'w') as f:
            json.dump(self.frames, f, indent=4)

        print(f"\nТраектория сохранена в {filename}")
        print(f"Записано фреймов: {len(self.frames)}")
        if self.frames:
            total_duration = sum(frame['duration'] for frame in self.frames)
            print(f"Общая длительность: {total_duration:.2f} сек")


def main():
    parser = argparse.ArgumentParser(
        description="Record G1 joint states from LowState topic"
    )
    parser.add_argument("network_interface", nargs="?", default="eth0")
    parser.add_argument("--topic", default="rt/lowstate")
    args = parser.parse_args()

    ChannelFactoryInitialize(0, args.network_interface)

    recorder = JointStateRecorder(topic=args.topic)
    recorder.start()

    print(f"Interface: {args.network_interface}")
    print(f"Topic: {args.topic}")

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

