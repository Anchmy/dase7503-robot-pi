# 地图

把场地图放在这里：

* `field.pgm` — Cartographer 或 slam_toolbox 生成的占据栅格
* `field.yaml` — 地图元数据（分辨率、原点、阈值）

`navigation.launch.py` 默认加载 `field.yaml`。

场地是 3020 x 2000 mm。0.05 m/像素时地图图像约 61 x 41 像素，再加一点围栏边距；4 x 3 m 的地图在 0.05 m/px 下（80 x 60 像素）是比较舒服的尺寸。
