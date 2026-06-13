# 火车驾驶游戏

## 技术栈
- Python >= 3.11, uv, pygame-ce 2.5.7
- MVC 架构: model / view / controller

## 轨道模型
- 三维坐标 (x, y, z), 本地米制, Z 暂存高程
- 图拓扑: Node + Edge, 无向
- Node 连接数: 1(尽头) 2(中间) 3(道岔) 4(交叉)
- 默认全互通, 连通性约束后续细化
- GeoJSON LineString 定义轨道:
  - 2 点 = 直线
  - 3 点 [A,B,C] = 圆弧 (minor arc), B 为 A/C 两切线的交点
  - ≥4 点 = 非法
  - 约束: A/B/C 不共线, |BA| = |BC|
- 圆弧 Edge 存储: center, radius, signed_angle, start_dir, normal
- sample_arc_points() 用于渲染, 模型层可直接使用曲线方程

## 渲染层
- pygame-ce 2D 调试视图
- Camera: 平移(左键拖拽)/缩放(滚轮), Y轴向上
- Renderer: 直线(灰色) 圆弧(蓝色) 节点(颜色按连接数)
- 运行: `uv run python main.py`
