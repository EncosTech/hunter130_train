# Encos130 Train

基于 [TienKung-Lab](https://github.com/Open-X-Humanoid/TienKung-Lab) 修改的 Encos130 人形机器人训练项目，使用 **Isaac Lab + PPO + AMP** 训练 23 自由度机器人的平地行走策略。

## 安装

当前使用 **Isaac Sim 5.1**。推荐使用 **Conda + pip**，按照 [官方安装指南](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html) 配置兼容的 Isaac Lab 环境。

激活环境后，在仓库根目录安装：

```bash
python -m pip install -e ./rsl_rl
python -m pip install -e .
python -m pip install scipy tensorboard
```

## 训练

以下命令均在仓库根目录执行：

```bash
python legged_lab/scripts/train.py --task encos130_walk --headless --num_envs 4096
```

显存不足时降低 `--num_envs`。日志和检查点保存在 `logs/encos130_walk/`。

```bash
tensorboard --logdir logs/encos130_walk
```

训练参数、奖励和参考动作路径见 [walk_cfg.py](legged_lab/envs/encos130/walk_cfg.py)。

## 回放与导出

将运行目录和检查点替换为实际值：

```bash
python legged_lab/scripts/play.py \
  --task encos130_walk --num_envs 1 \
  --load_run '<运行目录名>' --checkpoint model_1000.pt
```

回放时自动在对应运行目录的 `exported/` 下导出 `policy.pt` 和 `policy.onnx`。

## 致谢与许可证

感谢 [TienKung-Lab](https://github.com/Open-X-Humanoid/TienKung-Lab)、[Isaac Lab](https://github.com/isaac-sim/IsaacLab)、[RSL-RL](https://github.com/leggedrobotics/rsl_rl) 和 Legged Lab 的开源贡献。

本项目继续沿用上游 TienKung-Lab 的 [BSD-3-Clause 开源协议](LICENSE)，保留原有许可证及版权声明。模型和动作数据的授权以各自来源说明为准。
