双船 spawn 配置:告诉 launch 要起 wamv1 + wamv2 两只船
ros2 launch vrx_gz competition.launch.py config_file:=vrx_gz/config/two_wamv.yaml

wam-v 话题配置: wam-v 推进器/发球话题
单船手柄配置:摇杆/按钮 → wam-v 推进器/发球话题
roboboat.yaml roboboat 手柄配置:同上但话题名和力矩不同

