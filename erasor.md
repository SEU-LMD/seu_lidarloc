# OfflineMapUpdater

## 1.接受数据集以及内部定义的各种话题

## 2.发布话题

## 3.初始化指针

## 4.设置参数

## 5.加载全局地图：

### 5.1

     注意几个变量:
     1.map_init : 原始点云地图
     2.map_arranged_init :根据outdoor和indoor不同,户外就是等同于map_init；
     3.对于大场景静态地图，将map_arranged用作map_arranged_global_，而后续map_arranged将表示为子地图；
     4.map_arranged:是进行eraser的地图
     **note**:后续将会将map_arranged_global_分成会将map_arranged_init分成map_arranged和map_arranged_complement_，然后将map_arranged给到
     如果不是大场景静态地图，则

# OfflineMapUpdater::callback_node(*接收到的点云的消息放在这个回调函数)

## 1.设置VOI
### 1.1 设置当前帧的VOI
    1.1.1  对输入的点云进行降采样后从lidar系转换到车体坐标系
    1.1.2  然后将点云从车体坐标系
### 1.2 设置map的VOI
### 1.2.1  如果是大场景地图则需要构建局部地图
#### 1.2.1.1 对局部地图进行初始化
      1.将map_arranged_global_分成map_arranged_（距离当前帧位姿x和y200米以内的点云，即为局部地图）和map_arranged_complement_
      2.当激光所对应的位姿距离中心点超过100米时，则重新建立新的局部地图
#### 1.2.1.2 分配 VOI指针
      1.把输入的地图map_arranged将点根据距离分成地图中心map_central和地图边缘map_outskirts,两种模式
       naive:60米内是中心点，其余是边缘点
       kdtree:最近邻搜索60.5米内的点云，通过isTrue判断是否是边缘点还是中心点。
      2.将点云转换到本体坐标系
## 2. 将点云划分成Bin
### 2.1 将当前帧的点云VOI划分成Bin
     1.将VOI分成Bin,存索引，然后将符合条件的点云放到对应的Bin中，更新Bin的信息
### 2.2  将map的点云VOI划分成Bin
     1.  规定范围内的z与半径r的点划分成Bin,并将对应的点云存入Bin,范围之外的存入complement；
     2. 计算Bin的占用率（生成bin的各个顶点坐标，然后通过点云的z坐标的高度差计算占用率），将具有占用率的存入map_init；
     3. 计算map中bin的数目
## 3 SRT 判断那些有可能是动态点
### 3.1.naive
    1. 假如当前帧的点太少了，直接将当前帧所对应的地图点作为selected;
    2. 假如当前帧和地图都被占用
       2.1 则取二者Bin值更小的，当其小于0.22时，则将其列作动态Bin
       2.2 map_h大于curr_h的时候，将当前帧直接作为selected;提取地图中的地面点push到selected;非地面点作为rejected;
          2.2.1 将点云高度小于min_h的点云移去
          2.2.2 设置种子点，即计算最低的10个点的平均值，将低于平均值+阈值的点作为种子点返回
          2.2.3 通过SVD进行平面拟合；
          2.2.4 对于点到平面的距离小于阈值的点作为平面点
       2.3 curr_h大于map_h的时候，地图直接作为selected;地图点作为selected,当前帧的点放到rejected;
       2.4 当大于0.22.时，即作为selected;
    3.当只有当前帧是occupied时，当前帧就加进selected;
    4.当只有地图是occupied时，地图就加进selected;
    5.将selected的点作为地面点回收
    6.遍历selected的点，并分为有占用率的点和无占用率的点？

### 3.2 kd-tree
     1.与naive相似；
     2.唯一不同的地方在于对于Bin的比值大于0.2时，如果当前帧的Bin更高时，只将map中的点加入selected
     3.
## 4 滤除点
    1. 将rejected的点滤除 
    2.将点云转换到世界坐标系
# callback_flag
    1.回调完成后，将点云进行体素滤波然后保存