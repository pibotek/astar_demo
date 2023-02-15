# 编译
```sh
mkdir build
cd build
cmake .. # cmake -DDEBUG_PLANNER_DETAIL=ON ..
make
```

# 测试
```sh
./planner_test
```

# 说明
- 默认加载地图为`map/map_demo.png`
- 显示绿色为起点，红色是终点，如需修改，修改`main.cpp`中修改坐标点