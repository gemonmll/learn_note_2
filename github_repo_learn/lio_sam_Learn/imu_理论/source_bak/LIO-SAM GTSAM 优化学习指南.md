

# **LIO-SAM中GTSAM优化的深度解析：从理论到代码实现**

**引言：LIO-SAM与GTSAM的“天作之合”**

LIO-SAM (Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping) 是一个旨在实现高精度、实时机器人轨迹估计与建图的框架 1。它的核心特性在于后端优化，这一优化完全建立在GTSAM（Georgia Tech Smoothing and Mapping，佐治亚理工学院平滑与建图库）之上 4。GTSAM是一个专门为机器人和计算机视觉应用（包括SLAM）提供传感器融合与优化的C++库 5。

LIO-SAM的卓越性能，不仅源于其前端处理（例如利用IMU数据对点云进行去畸变 2），更关键的是它在后端**如何将复杂的SLAM问题“翻译”为GTSAM可以理解的数学语言**——即“因子图”（Factor Graph） 1。LIO-SAM的框架设计明确地将激光雷达-惯性里程计（LIO）构建在一个因子图之上，这允许系统将来自不同来源的大量相对测量（如Lidar里程计、IMU预积分）和绝对测量（如GPS、回环闭合）统一作为“因子”纳入优化系统 1。

本报告将深入剖析这一“翻译”与“优化”的过程，从支撑LIO-SAM的数学理论（因子图与最小二乘法）出发，详细解析LIO-SAM为实现实时性所设计的独特优化架构，最后通过具体的代码实例，逐一解构LIO-SAM中的关键因子（IMU、Lidar、GPS、回环）是如何在GTSAM中被创建、添加和优化的。

## **第一部分：理论基础——为什么选择因子图？**

在深入研究LIO-SAM的代码实现之前，必须首先理解其后端优化的数学和哲学基础。LIO-SAM（以及几乎所有现代高性能SLAM系统）选择因子图作为优化框架，是SLAM问题从“滤波”走向“平滑”的必然结果。

### **1.1 SLAM优化的演进：从滤波到平滑（Smoothing）**

LIO-SAM的名称（Lidar Inertial Odometry via *Smoothing* and Mapping）1 已经明确了其技术路线。

* **滤波（Filtering）**：传统的SLAM方法（如扩展卡尔曼滤波器 EKF-SLAM）采用“滤波”思想。它们只维护和估计*当前时刻*的系统状态（例如$X\_t$），历史状态（如$X\_{t-1}, X\_{t-2},...$）一旦估计完成，便被丢弃（或称边缘化）。这种方法计算效率高，但存在根本缺陷：它无法利用未来的信息去修正过去的错误。  
* **平滑（Smoothing）**：LIO-SAM采用的“平滑”方法（也称为“全SLAM”或“批量优化”）则完全不同。它维护一个包含*所有*历史状态（或一个时间窗口内）的估计值（例如$X\_1, X\_2,..., X\_t$）。

平滑方法之所以更优越，是因为新的测量信息可以反向传播，用于*修正过去*的状态。最典型的例子就是“回环闭合”（Loop Closure）1：当机器人在时刻$t$识别出它回到了时刻$j$（$j \\ll t$）的位置时，平滑方法可以将这个新的约束（$X\_t \\approx X\_j$）纳入系统，全局调整从$j$到$t$的*整段*轨迹，从而消除累积误差。LIO-SAM选择“平滑”是其能够实现高精度的先决条件，而因子图是实现“平滑”的最自然、最高效的数学工具。

### **1.2 因子图：SLAM问题的概率图模型**

GTSAM使用因子图和贝叶斯网络作为其底层的计算范式 5。因子图（Factor Graph）是一种二部图（Bipartite Graph），它由两类节点构成 8：

1. **变量节点（Variables）**：代表系统中待估计的未知随机变量 8。在LIO-SAM中，这通常包括机器人在不同时刻的位姿（Pose）、速度（Velocity）以及IMU的偏置（Bias）3。  
2. **因子节点（Factors）**：代表来自传感器测量或先验知识的*概率约束* 8。

LIO-SAM的整个后端就是围绕构建和维护这样一个因子图展开的 1。因子图不仅是一个数据结构，更是一种*建模哲学*。它将一个极其复杂的联合概率分布（即SLAM问题：给定所有测量$Z$，估计所有状态$X$的后验概率$P(X|Z)$）*分解*（Factorize）为一系列更简单的局部概率（因子）的乘积。

例如，一个Lidar里程计因子（Factor）可能只连接两个变量节点（Variables）：$X\_t$和$X\_{t+1}$，表示“$X\_t$和$X\_{t+1}$之间的相对位姿变换*应该等于*Lidar的测量值”。GTSAM之所以能高效利用稀疏性 8，是因为一个因子通常只连接*少数几个*变量。这种“稀疏性”是因子图相比传统（可能是稠密的）协方差矩阵方法的核心优势，也是LIO-SAM能实现实时的关键。

### **1.3 从因子图到非线性最小二乘**

因子图优化的目标是找到一组最可能的变量状态，即*最大后验概率*（Maximum a Posteriori, MAP）估计 3。

1. MAP的目标是最大化后验概率，这等价于最大化所有因子的乘积：

   $$X^\* \= \\text{argmax}\_X P(X|Z) \\propto \\text{argmax}\_X \\prod\_i \\phi\_i(X\_i)$$

   其中$X$是所有变量的集合，$Z$是所有测量，$\\phi\_i(X\_i)$是第$i$个因子，它只依赖于其连接的变量子集$X\_i$ 3。  
2. 在SLAM中，我们通常假设测量噪声服从高斯分布（Gaussian noise）。一个因子（例如，一个测量$z\_i$）可以表示为：

   $$\\phi\_i(X\_i) \\propto \\exp\\left(- \\frac{1}{2} \\|h\_i(X\_i) \- z\_i\\|^2\_{\\Sigma\_i}\\right)$$

   其中，$h\_i(X\_i)$是测量函数（即“预测值”），$z\_i$是实际测量值，$| \\cdot |^2\_{\\Sigma\_i}$表示马氏距离（Mahalanobis distance）。  
3. 为了最大化这个乘积，我们可以对其取负对数似然（Negative Log-Likelihood）10，这将乘法转换为加法，并将最大化问题转换为最小化问题：

   $$X^\* \= \\text{argmin}\_X \\sum\_i \\|h\_i(X\_i) \- z\_i\\|^2\_{\\Sigma\_i}$$

   这个公式就是我们熟知的非线性最小二乘（Non-linear Least Squares, NLLS）问题 12。

这个从MAP到NLLS的转换是理解GTSAM的核心。GTSAM中的每一个“因子”（Factor）的本质，就是一个*代价函数*（Cost Function）或*残差块*（Residual Block）。该因子定义了如何根据其连接的“变量”（Variables）计算一个*误差*（Error）或*残差*（Residual）。GTSAM的优化器（Optimizer），如列文伯格-马夸尔特（Levenberg-Marquardt）8 或高斯-牛顿（Gauss-Newton）12，的*唯一*工作，就是通过迭代（即重复地对非线性问题进行线性化 12）来调整所有变量的值，使得所有因子产生的误差（的马氏距离平方和）*最小*。

## **第二部分：LIO-SAM的优化架构与GTSAM的增量求解**

理论上，因子图可以解决SLAM问题，但在实践中，尤其是在Lidar和IMU这种高频数据流下，如果天真地构建一个包含*所有*测量和*所有*状态的图，其计算量将是灾难性的。LIO-SAM的精妙之处在于它为实现*实时性*所设计的*工程架构*。

### **2.1 LIO-SAM的实时性策略：双图架构**

LIO-SAM最核心的设计之一，是它维护了*两个*独立的因子图 4。

* **图 1 (高频图, 位于 imuPreintegration.cpp)**  
  * **职责**：优化IMU因子和激光雷达里程计因子 4。  
  * **目标**：在高频（IMU频率）下运行，主要用于*估计IMU偏置（Bias）* 2 并提供实时的、平滑的里程计估计。  
  * **特性**：此图是*滑动窗口*式的，并且会*周期性重置*（Reset）4。  
* **图 2 (全局图, 位于 mapOptimization.cpp)**  
  * **职责**：优化激光雷达里程计因子、GPS因子 4 和回环因子 1。  
  * **目标**：维护全局地图的*一致性*，消除长期漂移 7。  
  * **特性**：此图在整个系统的运行过程中*持续维护* 4，不被重置。

这个双图架构是一个精妙的*工程权衡*。一个单一的、在每次IMU测量时都更新的全局图（像某些VIO算法）在计算上是不可行的。LIO-SAM的作者在GitHub的讨论中明确了这一点：将IMU因子从主图（全局图）中分离出来，是为了“更快的优化”（faster optimization），尽管这可能会“稍微损害性能”（hurts performance a bit）17。

这个高频图（imuPreintegration.cpp）实质上扮演了一个*快速的、局部的、滑动窗口式的状态估计器*。它有两个核心产出：

1. 为前端提供高频运动估计，用于*点云去畸变*（Deskew）2。  
2. 生产出一个*高质量的IMU偏置估计* 2。

这个偏置估计*随后被用作“全局图”中IMU预积分的先验*。因此，高频图起到了“预处理”和“状态估计”的双重作用，极大地减轻了全局图的计算负担。

### **2.2 GTSAM的iSAM2：LIO-SAM的增量平滑核心**

LIO-SAM的全局图（mapOptimization.cpp）在插入新节点时，使用GTSAM中的iSAM2（Incremental Smoothing and Mapping）算法进行优化 18。

iSAM2 3 是一种*增量*（Incremental）推理算法。它与*批量*（Batch）优化（即每次添加新数据时，都从头解决整个NLLS问题 8）有着本质区别。iSAM2在添加新测量（因子）和新变量时，*不需要*重新计算所有内容。

iSAM2基于一种称为*贝叶斯树*（Bayes Tree）的数据结构 11。当新的测量（因子）被加入时，iSAM2能智能地识别出这些测量“通常只影响树的小部分，并且只有这些部分被重新计算” 11。它还能自动确定图中哪些旧的变量由于非线性误差的累积而需要被*重新线性化*（Relinearization）20。

LIO-SAM选择GTSAM的*根本原因*很可能就是为了使用iSAM2。如果LIO-SAM在每个关键帧都执行一次*批量*优化，系统将无法实时运行 12。iSAM2提供的update()函数 8 提供了“增量更新解”的能力，这与SLAM系统“不断有新数据流入”的增量运行特性完美契合。

然而，LIO-SAM的实时性是*两种策略*共同作用的结果：

1. **GTSAM的iSAM2**：提供了*高效的增量求解器*。  
2. **LIO-SAM的图管理策略**：LIO-SAM并非无限制地向iSAM2添加因子。它通过*边缘化*（Marginalization）旧的激光雷达扫描 2 和使用*局部地图*（而非全局地图）进行扫描匹配 2，主动地控制了因子图的*规模*。

这意味着LIO-SAM始终只让iSAM2处理一个*规模受限*的优化问题，从而保证了系统的实时性能。

## **第三部分：【实例解析】LIO-SAM因子在GTSAM中的代码实现**

本部分是报告的核心。我们将逐一分析LIO-SAM中构建的四种主要GTSAM因子，将理论与LIO-SAM源码（特别是imuPreintegration.cpp 22 和 mapOptimization.cpp 23）中的代码行联系起来。

为了清晰地概览这一过程，下表总结了LIO-SAM中的主要约束、它们在GTSAM中对应的C++类以及它们在LIO-SAM项目中的实现位置。

---

**表1：LIO-SAM中GTSAM因子的解构**

| 传感器/约束类型 | 对应的GTSAM因子 (GTSAM Factor Class) | 目的 (Purpose) | LIO-SAM实现文件 | 关键代码示例（概念化） |
| :---- | :---- | :---- | :---- | :---- |
| **IMU预积分** | gtsam::ImuFactor 22 | 在两个Lidar关键帧之间，紧耦合地约束位姿、速度和偏置。 | imuPreintegration.cpp | graphFactors.add(ImuFactor(X(k-1), V(k-1), X(k), V(k), B(k-1), preint\_imu)); |
| **IMU偏置** | gtsam::BetweenFactor\<gtsam::imuBias::ConstantBias\> 24 | 约束连续关键帧之间的IMU偏置，假设其缓慢变化（随机游走）。 | imuPreintegration.cpp | graphFactors.add(BetweenFactor\<...Bias\>(B(k-1), B(k),...)); |
| **激光雷达里程计** | gtsam::BetweenFactor\<gtsam::Pose3\> | 约束两个连续关键帧之间的相对位姿变换。 | mapOptimization.cpp | graphFactors.add(BetweenFactor\<Pose3\>(X(k-1), X(k), relative\_pose, noise)); |
| **GPS绝对定位** | gtsam::PriorFactor\<gtsam::Pose3\> 22 | 提供一个全局绝对位置约束，锚定地图，修正漂移。 | mapOptimization.cpp | graphFactors.add(PriorFactor\<Pose3\>(X(k), gps\_pose, gps\_noise)); |
| **回环闭合** | gtsam::BetweenFactor\<gtsam::Pose3\> 8 | 约束当前关键帧与一个*历史*关键帧之间的相对位姿，消除累积误差。 | mapOptimization.cpp | graphFactors.add(BetweenFactor\<Pose3\>(X(j), X(k), loop\_relative\_pose, loop\_noise)); |

---

### **3.1 IMU预积分因子 (gtsam::ImuFactor)**

**理论**：IMU测量（角速度$\\hat{\\omega}\_t$和加速度$\\hat{a}\_t$）18 的频率（例如200Hz）远高于Lidar（例如10Hz）。*IMU预积分*（IMU Preintegration）3 是一种数学技巧，它将两个Lidar关键帧（例如$k-1$和$k$）之间的*所有*高频IMU测量*积分*成一个*单一*的相对运动约束（$\\Delta R, \\Delta v, \\Delta p$）。这个约束同时还是IMU偏置（Bias）的函数，这使得在优化过程中可以同时估计运动和偏置 2。

**GTSAM API**：GTSAM为此提供了两个关键类：

1. gtsam::PreintegratedImuMeasurements 26：这是一个“累积器”。在两个关键帧之间，每当有新的IMU数据到来，就调用它的integrateMeasurement方法将其累积起来。  
2. gtsam::ImuFactor 22：这是最终被添加到因子图中的*约束*（即代价函数）。

**LIO-SAM代码 (imuPreintegration.cpp)**：

1. **初始化积分器**：在imuPreintegration.cpp中，会创建一个积分器对象。  
   C++  
   //   
   // p 包含了IMU的噪声参数  
   // prior\_imu\_bias 是当前对偏置的最佳估计  
   imuIntegratorImu\_ \= new gtsam::PreintegratedImuMeasurements(p, prior\_imu\_bias); 

2. **创建因子**：当一个新的Lidar关键帧key到来时，LIO-SAM会从积分器中取出积分结果preint\_imu，并创建ImuFactor。  
   C++  
   // \[22, 24, 33\]  
   // preint\_imu 是包含了k-1到k之间所有IMU测量的预积分对象  
   gtsam::ImuFactor imu\_factor(X(key \- 1), V(key \- 1),   
                               X(key),     V(key),   
                               B(key \- 1), preint\_imu);

3. **分析**：这行代码是“紧耦合”（Tightly-coupled）2 的核心体现。ImuFactor是一个*多元*因子，它*同时*连接了6个变量节点：  
   * X(key \- 1): 上一关键帧的位姿 (GTSAM中的Pose3)  
   * V(key \- 1): 上一关键帧的速度 (GTSAM中的Vector3)  
   * X(key): 当前关键帧的位姿 (GTSAM中的Pose3)  
   * V(key): 当前关键帧的速度 (GTSAM中的Vector3)  
   * B(key \- 1): 上一关键帧的IMU偏置 (GTSAM中的imuBias::ConstantBias)  
   * preint\_imu: 这是“测量值”。  
4. **添加因子**：graphFactors.add(imu\_factor); 24。  
5. **偏置约束**：LIO-SAM还假设IMU偏置是缓慢变化的（随机游走）。因此，它还会添加一个因子来约束*偏置本身*的变化：  
   C++  
   //   
   graphFactors.add(gtsam::BetweenFactor\<gtsam::imuBias::ConstantBias\>(B(key \- 1), B(key),   
                    gtsam::imuBias::ConstantBias(), bias\_noise\_model));

当GTSAM优化此图时，它不仅会调整位姿$X$和速度$V$，还会*反向传播*误差，以*更新*对偏置$B$的估计。这就是LIO-SAM如何“利用获得的激光雷达里程计解来估计IMU的偏置” 2 的答案。

### **3.2 激光雷达里程计因子 (gtsam::BetweenFactor)**

**理论**：LIO-SAM的前端（Lidar Odometry）27 会计算当前Lidar关键帧$k$与*局部地图*（由之前的“子关键帧” sub-keyframes 组成 2）之间的相对位姿变换 $T\_{k, map}$ 27。

**GTSAM API**：在GTSAM中，两个位姿之间的相对变换约束，是用gtsam::BetweenFactor\<gtsam::Pose3\>来建模的。

LIO-SAM代码 (mapOptimization.cpp)：  
当一个新的关键帧被添加到mapOptimization.cpp中的全局图时，会添加一个BetweenFactor来连接这个新位姿节点X(k)和它在局部地图中的“锚点”（可能是X(k-1)）。

C++

// \[34\]  
// lidar\_odometry\_measurement 是前端计算得到的 Pose3  
// lidar\_noise\_model 是该测量的协方差  
graphFactors.add(gtsam::BetweenFactor\<gtsam::Pose3\>(X(key\_prev), X(key\_curr),   
                 lidar\_odometry\_measurement, lidar\_noise\_model));

LIO-SAM的*实时性*秘诀之一在于，它*不*是“scan-to-global-map”（扫描到全局地图）匹配 2。它只进行“scan-to-local-map”（扫描到局部地图）匹配 2。这意味着计算Lidar里程计因子（即lidar\_odometry\_measurement）的*计算量是恒定的*，不会随地图增大而增大。

### **3.3 GPS因子 (gtsam::PriorFactor)**

**理论**：GPS（或更广义的GNSS）提供了关于机器人*绝对*位置的测量 7。这对于修正Lidar和IMU积分产生的*长期漂移*至关重要 7。LIO-SAM以松耦合（loosely coupled）的方式融合GPS数据 29。

**GTSAM API**：对*单个*变量的绝对测量（或先验知识）在GTSAM中被称为“先验”（Prior），使用gtsam::PriorFactor\<T\>建模。对于一个3D位姿，这就是gtsam::PriorFactor\<gtsam::Pose3\>。

**LIO-SAM代码 (mapOptimization.cpp)**：

1. **图的锚定**：在图的*最开始*（key \= 0）或在*重置*（reset）之后，LIO-SAM必须添加一个先验因子来“锚定”整个坐标系，否则图是“漂浮”的、不可解的。  
   C++  
   //   
   // X(0)是第一个位姿变量，prevPose\_是初始位姿（如(0,0,0)）  
   // priorPoseNoise 是初始位姿的协方差  
   gtsam::PriorFactor\<gtsam::Pose3\> priorPose(X(0), prevPose\_, priorPoseNoise);  
   graphFactors.add(priorPose);

   22还显示了在重置图时，也会为速度V(0)和偏置B(0)添加相应的PriorFactor。  
2. **融合GPS**：当一个GPS测量到来时，LIO-SAM会将其转换为一个Pose3类型的测量gps\_pose，并为*当前*的关键帧key添加一个新的PriorFactor：  
   C++  
   // \[24, 29\]  
   gtsam::noiseModel::Diagonal::shared\_ptr gps\_noise \=...; // 基于GPS的协方差  
   gtsam::PriorFactor\<gtsam::Pose3\> gpsFactor(X(key), gps\_pose, gps\_noise);  
   graphFactors.add(gpsFactor);

PriorFactor和BetweenFactor的区别至关重要。BetweenFactor连接*两个*变量，约束它们的*相对*关系。PriorFactor只连接*一个*变量，约束它的*绝对*值。这就是“松耦合” 29 的含义：GPS的测量*直接*约束了状态变量X(key)。这种方法的*风险*在于，如果GPS信号*错误*（如在隧道或城市峡谷 28），一个高置信度（低噪声）的PriorFactor会*强行*将位姿拉到一个错误的位置，导致优化结果*崩溃* 28。

### **3.4 回环因子 (gtsam::BetweenFactor)**

**理论**：回环检测 1 是指系统识别出它*回到了*一个以前去过的地方。LIO-SAM提供了一个基于LeGO-LOAM的回环检测示例，并推荐使用ScanContext等更先进的方法 4。当检测到回环时（例如，当前位姿X(100)被识别出与历史位姿X(5)在同一地点），系统会计算一个*相对位姿* $T\_{100, 5}$。

**GTSAM API**：这仍然是一个*相对位姿*约束，因此它与Lidar里程计因子一样，使用gtsam::BetweenFactor\<gtsam::Pose3\> 8。

LIO-SAM代码 (mapOptimization.cpp)：  
当回环检测模块（4）触发时，它会找到当前的key\_current和匹配的key\_matched，并通过ICP等方式计算它们之间的相对位姿relative\_pose。然后，它会向图中添加一个回环因子（Loop Closure Factor）9：

C++

//   
// 假设当前是 k=100, 匹配到 j=5  
gtsam::noiseModel::Diagonal::shared\_ptr loop\_noise \=...; // 回环的噪声  
graphFactors.add(gtsam::BetweenFactor\<gtsam::Pose3\>(X(5), X(100),   
                 relative\_pose, loop\_noise));

Lidar*里程计*因子和Lidar*回环*因子在GTSAM中*使用相同的类* (BetweenFactor)。它们唯一的区别是：里程计因子连接的是*时间上相邻*的节点（如X(99)和X(100)）30，而回环因子连接的是*时间上相距甚远*的节点（如X(5)和X(100)）8。

在因子图中添加这个“跨越时空”的回环因子，是*修正累积误差*的唯一手段。Lidar里程计因子像链条一样一节扣一节，时间久了必然会*漂移*。而回环因子就像一个*订书钉*，将链条的末端（X(100)）和开端（X(5)）*强行*钉在了一起。当iSAM2求解这个“钉住”了的图时，误差会沿着整个链条*反向传播*并被*分摊*，从而实现全局轨迹的校正。

## **第四部分：工作流综合——LIO-SAM中一次GTSAM优化周期的完整步进**

本部分将把所有概念串联起来，模拟mapOptimization.cpp（或imuPreintegration.cpp）中一个GTSAM优化周期的完整步进。

### **Step 1: 初始化 (Initialization)**

* **动作**：系统启动，iSAM2对象被创建 19。  
* **动作**：第一个关键帧到来，key \= 0。  
* **GTSAM操作**：  
  1. 创建变量 X(0)（位姿）、V(0)（速度）、B(0)（偏置）。  
  2. 向graphValues中插入这些变量的初始值（例如，(0,0,0)）8。  
  3. 向graphFactors中添加*先验因子*（Priors）来“锚定”图：PriorFactor\<Pose3\>(X(0),...) 24, PriorFactor\<Vector3\>(V(0),...) 22, PriorFactor\<imuBias::ConstantBias\>(B(0),...) 22。  
  4. 调用 isam.update(graphFactors, graphValues); 8。

### **Step 2: 增量更新 (Incremental Update)**

* **动作**：新关键帧key \= k到来。前端计算了Lidar里程计，IMU预积分模块也准备好了preint\_imu对象。  
* **GTSAM操作**：  
  1. 清空临时的graphFactors和graphValues（isam.update后它们就被清空了）。  
  2. 创建新变量 X(k), V(k), B(k)。  
  3. 向graphValues中插入这些新变量的*初始估计值*（Initial Estimate）8。这个初始估计值非常重要，它通常来自IMU的预积分结果 2 或Lidar里程计的估计。GTSAM的非线性优化*需要*一个良好的初始值才能快速收敛。  
  4. 向graphFactors中添加*新*的因子：  
     * ImuFactor(X(k-1), V(k-1), X(k), V(k), B(k-1),...) 22  
     * BetweenFactor\<...Bias\>(B(k-1), B(k),...) 24  
     * BetweenFactor\<Pose3\>(X(k-1), X(k),...) (Lidar里程计)  
  5. 调用 isam.update(graphFactors, graphValues); 8。iSAM2会高效地求解*更新后*的贝叶斯树 11，得到*所有*变量（从X(0)到X(k))的最新估计。  
  6. 从isam.calculateEstimate() 8 中提取优化后的位姿result.at\<Pose3\>(X(k)) 8 作为该关键帧的最终位姿。

### **Step 3: 绝对测量 (Absolute Measurement)**

* **动作**：在Step 2中，如果恰好有GPS数据可用。  
* **GTSAM操作**：在向graphFactors添加Lidar和IMU因子的*同时*，*额外*添加一个 PriorFactor\<Pose3\>(X(k), gps\_pose, gps\_noise) 29。  
* **动作**：然后*同样*调用isam.update()。iSAM2的update调用是*原子*的。LIO-SAM将*所有*在两次update之间新产生的因子（Lidar, IMU, GPS）*打包*在一起，一次性提交给iSAM2进行优化。

### **Step 4: 全局闭环 (Global Loop Closure)**

* **动作**：系统检测到X(k)与历史帧X(j) ($j \\ll k$) 构成回环 4。  
* **GTSAM操作**：  
  1. 清空graphFactors和graphValues。  
  2. 向graphFactors中添加一个*单独*的 BetweenFactor\<Pose3\>(X(j), X(k), loop\_relative\_pose, loop\_noise) 8。  
  3. 调用 isam.update(graphFactors, graphValues);（此时graphValues为空，因为没有新变量）。  
* **分析**：这次update调用的*计算量会非常大*。因为这个新因子连接了树的两个遥远分支（X(j)和X(k)），它将导致iSAM2贝叶斯树的大规模重构和重线性化 11。这是实现全局一致性所*必须*付出的计算代价，也是iSAM2相比批量优化（需要优化*一切*）的*巨大*优势（iSAM2仍然只优化受影响的部分）。

### **Step 5: 图的维护 (Graph Maintenance)**

* **动作**：当图变得太大时（例如在imuPreintegration.cpp中key \== 100 22）。  
* **GTSAM操作**：LIO-SAM的imuPreintegration.cpp展示了一个*重置*（Reset）策略，这本质上是一种*边缘化*（Marginalization）2。  
  1. 使用optimizer.marginalCovariance(X(key \- 1)) 22 获取当前状态（位姿、速度、偏置）的*边缘协方差*。  
  2. 调用resetOptimization() 22，清空iSAM2优化器和所有因子。  
  3. 将key-1的状态和其协方差作为*新的先验*（PriorFactor）22 添加到*空*的因子图中。  
* **分析**：系统将所有0到k-1的历史信息*总结*成一个PriorFactor，然后丢弃了所有0到k-1的因子和变量，从而释放了内存并保证了后续的计算速度。

## **第五部分：专家洞察与结论**

### **5.1 LIO-SAM设计决策的权衡**

LIO-SAM的成功 1 很大程度上源于其*务实*的工程决策。双图架构 4 就是一个典型例子。这是一个为了*速度*（real-time）而牺牲*理论最优性*（如开发者所承认的"hurts performance a bit" 17）的权衡。

* 高频、周期重置的imuPreintegration图 4 确保了实时的IMU偏置估计和点云去畸变 2。  
* 低频、全局一致的mapOptimization图 4 则利用iSAM2 18 确保了长期的准确性。

这种“分而治之”的策略是LIO-SAM的核心竞争力。

### **5.2 基于GTSAM的扩展性**

LIO-SAM的因子图框架 1 具有天然的*可扩展性*。GTSAM 5 使得添加*新类型*的传感器测量变得异常简单：只需要定义一个*新的因子类*（Custom Factor）30 来描述新传感器的测量模型（即它的代价函数）即可。

LB-LIOSAM 9 就是一个绝佳的例证。它在LIO-SAM的*框架*基础上，通过集成LinK3D特征 9 来生成*更鲁棒*的“LiDAR测量因子”，并集成BoW3D 9 来生成*更准确*的“回环因子”。它*没有*重写优化器，它只是向GTSAM的图中添加了*质量更高*的因子，GTSAM的iSAM2会自动处理后续的优化。

### **5.3 最终结论**

LIO-SAM利用GTSAM，将复杂的SLAM问题成功地*分解*（Factorize）为四个核心的因子类型：IMU预积分、Lidar里程计、GPS先验和回环。

它*不是*一个简单的GTSAM“用户”。它通过*双图架构* 4、*局部地图匹配* 2 和*边缘化* 2 等策略，聪明地*管理*了提交给GTSAM的因子图的*规模*。

最终，LIO-SAM的实时高性能，是GTSAM强大的*增量非线性优化器*（iSAM2）18 与LIO-SAM巧妙的*图管理策略* 2 共同作用的产物。学习LIO-SAM中GTSAM的应用，就是学习如何将传感器模型（如IMU运动学）“翻译”为GTSAM中的因子（如ImuFactor），以及如何设计一个系统来*实时*地、*高效*地构建和维护这个因子图。

#### **Works cited**

1. LIO-SAM \- Autoware Documentation, accessed November 13, 2025, [https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-maps/open-source-slam/lio-sam/](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-maps/open-source-slam/lio-sam/)  
2. LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping \- arXiv, accessed November 13, 2025, [https://arxiv.org/abs/2007.00258](https://arxiv.org/abs/2007.00258)  
3. LIO-GC: LiDAR Inertial Odometry with Adaptive Ground Constraints \- MDPI, accessed November 13, 2025, [https://www.mdpi.com/2072-4292/17/14/2376](https://www.mdpi.com/2072-4292/17/14/2376)  
4. LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping \- GitHub, accessed November 13, 2025, [https://github.com/TixiaoShan/LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)  
5. GTSAM | GTSAM is a BSD-licensed C++ library that implements sensor fusion for robotics and computer vision using factor graphs., accessed November 13, 2025, [https://gtsam.org/](https://gtsam.org/)  
6. LIO-SAM \- Autoware Documentation, accessed November 13, 2025, [https://autowarefoundation.github.io/autoware-documentation/pr-493/integration/creating-maps/lio-sam/](https://autowarefoundation.github.io/autoware-documentation/pr-493/integration/creating-maps/lio-sam/)  
7. A Review of Multi-Sensor Fusion SLAM Systems Based on 3D LIDAR \- MDPI, accessed November 13, 2025, [https://www.mdpi.com/2072-4292/14/12/2835](https://www.mdpi.com/2072-4292/14/12/2835)  
8. Factor Graphs and GTSAM, accessed November 13, 2025, [https://gtsam.org/tutorials/intro.html](https://gtsam.org/tutorials/intro.html)  
9. LB-LIOSAM: an improved mapping and localization method with loop detection \- Emerald, accessed November 13, 2025, [https://www.emerald.com/ir/article/52/3/381/1272508/LB-LIOSAM-an-improved-mapping-and-localization](https://www.emerald.com/ir/article/52/3/381/1272508/LB-LIOSAM-an-improved-mapping-and-localization)  
10. Factor Graphs in Optimization-Based Robotic Control—A Tutorial and Review \- IEEE Xplore, accessed November 13, 2025, [https://ieeexplore.ieee.org/iel8/6287639/10820123/10855417.pdf](https://ieeexplore.ieee.org/iel8/6287639/10820123/10855417.pdf)  
11. iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree \- CMU School of Computer Science, accessed November 13, 2025, [https://www.cs.cmu.edu/\~kaess/pub/Kaess12ijrr.pdf](https://www.cs.cmu.edu/~kaess/pub/Kaess12ijrr.pdf)  
12. Factor Graphs for Navigation Applications: A Tutorial, accessed November 13, 2025, [https://navi.ion.org/content/71/3/navi.653](https://navi.ion.org/content/71/3/navi.653)  
13. Introduction to Factor Graph — miniSAM 0.1 documentation, accessed November 13, 2025, [https://minisam.readthedocs.io/factor\_graph.html](https://minisam.readthedocs.io/factor_graph.html)  
14. miniSAM: A Flexible Factor Graph Non-linear Least Squares Optimization Framework, accessed November 13, 2025, [https://project.inria.fr/ppniv19/files/2019/11/PPNIV19-paper\_Dong.pdf](https://project.inria.fr/ppniv19/files/2019/11/PPNIV19-paper_Dong.pdf)  
15. README.md · master · Siyuan Ren / LIO-SAM \- GitLab, accessed November 13, 2025, [https://gitlab.uni-hannover.de/esistoiamk/LIO-SAM/-/blob/master/README.md](https://gitlab.uni-hannover.de/esistoiamk/LIO-SAM/-/blob/master/README.md)  
16. LIO-SAM \- Siyuan Ren \- GitLab, accessed November 13, 2025, [https://gitlab.uni-hannover.de/esistoiamk/LIO-SAM](https://gitlab.uni-hannover.de/esistoiamk/LIO-SAM)  
17. Why use two factor graph? · Issue \#51 · TixiaoShan/LIO-SAM \- GitHub, accessed November 13, 2025, [https://github.com/TixiaoShan/LIO-SAM/issues/51](https://github.com/TixiaoShan/LIO-SAM/issues/51)  
18. LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping \- MIT Senseable City Lab, accessed November 13, 2025, [https://senseable.mit.edu/papers/pdf/20201020\_Shan-etal\_LIO-SAM\_IROS.pdf](https://senseable.mit.edu/papers/pdf/20201020_Shan-etal_LIO-SAM_IROS.pdf)  
19. Implementation of ROS-based Multi-Agent SLAM Centralized and Decentralized Approaches \- POLITECNICO DI TORINO, accessed November 13, 2025, [https://webthesis.biblio.polito.it/29366/1/tesi.pdf](https://webthesis.biblio.polito.it/29366/1/tesi.pdf)  
20. iSAM: Incremental Smoothing and Mapping — GTSAM 4.0.2 documentation, accessed November 13, 2025, [https://gtsam-jlblanco-docs.readthedocs.io/en/latest/iSAM.html](https://gtsam-jlblanco-docs.readthedocs.io/en/latest/iSAM.html)  
21. MIT Open Access Articles iSAM2: Incremental Smoothing and Mapping with Fluid Relinearization and Incremental Variable Reordering \- DSpace@MIT, accessed November 13, 2025, [https://dspace.mit.edu/bitstream/handle/1721.1/64749/Leonard\_iSAM2%20Incremental.pdf?sequence=1\&isAllowed=y](https://dspace.mit.edu/bitstream/handle/1721.1/64749/Leonard_iSAM2%20Incremental.pdf?sequence=1&isAllowed=y)  
22. LIO-SAM代码学习——imuPreintegration.cpp 原创 \- CSDN博客, accessed November 13, 2025, [https://blog.csdn.net/qq\_44043868/article/details/122705165](https://blog.csdn.net/qq_44043868/article/details/122705165)  
23. LIO-SAM学习总结原创 \- CSDN博客, accessed November 13, 2025, [https://blog.csdn.net/qq\_44709827/article/details/130983871](https://blog.csdn.net/qq_44709827/article/details/130983871)  
24. SLAM学习笔记（二十）LIO-SAM流程及代码详解（最全） 原创 \- CSDN博客, accessed November 13, 2025, [https://blog.csdn.net/zkk9527/article/details/117957067](https://blog.csdn.net/zkk9527/article/details/117957067)  
25. 4.1. Inertial Estimation with Imu Preintegration — GTSAM by Example, accessed November 13, 2025, [https://gtbook.github.io/gtsam-examples/ImuFactorExample101.html](https://gtbook.github.io/gtsam-examples/ImuFactorExample101.html)  
26. Compiling LIO-SAM on Ubuntu 22.04 and ROS2 · Issue \#400 \- GitHub, accessed November 13, 2025, [https://github.com/TixiaoShan/LIO-SAM/issues/400](https://github.com/TixiaoShan/LIO-SAM/issues/400)  
27. LIO-SAM++: A Lidar-Inertial Semantic SLAM with Association Optimization and Keyframe Selection \- PMC \- NIH, accessed November 13, 2025, [https://pmc.ncbi.nlm.nih.gov/articles/PMC11644182/](https://pmc.ncbi.nlm.nih.gov/articles/PMC11644182/)  
28. GNSS/Multi-Sensor Fusion Using Continuous-Time Factor Graph Optimization for Robust Localization \- arXiv, accessed November 13, 2025, [https://arxiv.org/html/2309.11134v2](https://arxiv.org/html/2309.11134v2)  
29. Issue \#417 · TixiaoShan/LIO-SAM \- GPS Implementation \- GitHub, accessed November 13, 2025, [https://github.com/TixiaoShan/LIO-SAM/issues/417](https://github.com/TixiaoShan/LIO-SAM/issues/417)  
30. GTSAM 4.0 Tutorial Theory, Programming, and Applications \- Jing Dong, accessed November 13, 2025, [https://dongjing3309.github.io/files/gtsam-tutorial.pdf](https://dongjing3309.github.io/files/gtsam-tutorial.pdf)  
31. Factor Graphs and GTSAM: A Hands-on Introduction \- GT Digital Repository, accessed November 13, 2025, [https://repository.gatech.edu/bitstreams/b3606eb4-ce55-4c16-8495-767bd46f0351/download](https://repository.gatech.edu/bitstreams/b3606eb4-ce55-4c16-8495-767bd46f0351/download)  
32. Tutorials — GTSAM 4.0.2 documentation \- Read the Docs, accessed November 13, 2025, [https://gtsam-jlblanco-docs.readthedocs.io/en/latest/Tutorials.html](https://gtsam-jlblanco-docs.readthedocs.io/en/latest/Tutorials.html)