#### 根据官方文档学习使用Ceres:

​	http://www.ceres-solver.org/nnls_tutorial.html

#### 笔记：

​	https://www.yuque.com/tengjialin/slam/lmuotlv1h0qvtbkv

#### floam中对于点线误差的解析式求导要注意：

​	通过SizeCostFunction 和LocalParameterization 实现旋转过程中显示se3的求导
对于delta_x增量非常小的时候，需要对Quaterniond 的虚部系数和 J进行泰勒展开和化简

关于矩阵求导参考链接：

https://www.yuque.com/tengjialin/slam/gmtg97sxn143flcy
