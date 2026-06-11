

# **深度解析：LIO-SAM中的IMU系统公式理论与IMU预积分原理**

## **1\. 惯性导航基础：运动学模型与噪声模型**

要深入理解LIO-SAM中的惯性测量单元（IMU）系统，必须首先建立一个坚实的理论基础，该基础始于对IMU测量数据的精确数学建模。这包括定义状态变量、运动学方程以及随机噪声模型。

### **1.1 坐标系与状态定义**

在任何传感器融合问题中，首先必须明确定义坐标系。

* **世界坐标系 ($W$)**: 这是一个固定的惯性参考系。在此框架下，重力向量 $\\mathbf{g}\_W$ 是恒定的，通常定义为 $\\mathbf{g}\_W \= \[0, 0, \-9.81\]^T$ 1。  
* **IMU/机体坐标系 ($B$)**: 这是一个非惯性系，刚性地附着在IMU传感器上 1。所有IMU测量都在此坐标系中进行。  
* **激光雷达坐标系 ($L$)**: 这是一个刚性地附着在LiDAR传感器上的坐标系。LIO-SAM要求将IMU原始数据从IMU坐标系转换到LiDAR坐标系 2。

系统的完整状态 $X$ 在任意时刻 $t$ 由其在世界坐标系中的姿态、位置、速度以及IMU传感器的内部偏置（bias）共同定义 3。状态向量 $X(t)$ 表示为：

$$X(t) \= \\{ R\_{WB}(t), \\mathbf{p}\_W(t), \\mathbf{v}\_W(t), \\mathbf{b}\_a(t), \\mathbf{b}\_g(t) \\}$$  
其中：

* $R\_{WB}(t) \\in SO(3)$ 是一个 $3 \\times 3$ 的旋转矩阵，用于将向量从机体坐标系 $B$ 转换到世界坐标系 $W$。  
* $\\mathbf{p}\_W(t) \\in \\mathbb{R}^3$ 是机体坐标系 $B$ 的原点在世界坐标系 $W$ 中的位置。  
* $\\mathbf{v}\_W(t) \\in \\mathbb{R}^3$ 是机体坐标系 $B$ 的原点在世界坐标系 $W$ 中的速度。  
* $\\mathbf{b}\_a(t) \\in \\mathbb{R}^3$ 和 $\\mathbf{b}\_g(t) \\in \\mathbb{R}^3$ 分别是加速度计和陀螺仪的偏置向量，它们被建模为缓慢时变的状态 3。

### **1.2 IMU测量模型（随机模型）**

IMU（通常为6轴）并不直接测量姿态或位置；它测量的是角速度和比力（specific force）4。这些测量值会受到传感器偏置和随机噪声的污染。

陀螺仪测量 ($\\tilde{\\boldsymbol{\\omega}}$)  
陀螺仪测量的角速度 $\\tilde{\\boldsymbol{\\omega}}(t)$ 是真实角速度 $\\boldsymbol{\\omega}(t)$（$B$ 系相对于 $W$ 系的角速度，在 $B$ 系中表示）、一个缓慢变化的偏置 $\\mathbf{b}\_g(t)$ 和高斯白噪声 $\\mathbf{n}\_g(t)$ 的总和 4。  
$$\\tilde{\\boldsymbol{\\omega}}(t) \= \\boldsymbol{\\omega}(t) \+ \\mathbf{b}\_g(t) \+ \\mathbf{n}\_g(t), \\quad \\mathbf{n}\_g(t) \\sim \\mathcal{N}(0, \\sigma\_g^2 \\mathbf{I})$$  
加速度计测量 ($\\tilde{\\mathbf{a}}$)  
这是一个关键点：加速度计测量的不是真实的运动学加速度，而是“比力”，即非引力引起的加速度。因此，测量值 $\\tilde{\\mathbf{a}}(t)$ 等于真实的运动学加速度 $\\mathbf{a}(t)$（$B$ 系相对于 $W$ 系的加速度，在 $B$ 系中表示）减去投影到 $B$ 系的重力加速度，再加上偏置 $\\mathbf{b}\_a(t)$ 和高斯白噪声 $\\mathbf{n}\_a(t)$ 4。  
$$\\tilde{\\mathbf{a}}(t) \= \\mathbf{a}(t) \+ R\_{BW}(t) \\mathbf{g}\_W \+ \\mathbf{b}\_a(t) \+ \\mathbf{n}\_a(t), \\quad \\mathbf{n}\_a(t) \\sim \\mathcal{N}(0, \\sigma\_a^2 \\mathbf{I})$$  
这里，$R\_{BW}(t) \= R\_{WB}(t)^T$ 是从世界坐标系 $W$ 到机体坐标系 $B$ 的旋转矩阵。

偏置（Bias）的随机游走模型  
对IMU偏置最有效且最常用的建模方式是将其视为一个随机游走（Random Walk）或维纳过程（Wiener process）4。这意味着偏置的时间导数（即变化率）本身是一个高斯白噪声过程：  
$$\\dot{\\mathbf{b}}\_g(t) \= \\mathbf{n}\_{wg}(t), \\quad \\mathbf{n}\_{wg}(t) \\sim \\mathcal{N}(0, \\sigma\_{bg}^2 \\mathbf{I})$$  
$$\\dot{\\mathbf{b}}\_a(t) \= \\mathbf{n}\_{wa}(t), \\quad \\mathbf{n}\_{wa}(t) \\sim \\mathcal{N}(0, \\sigma\_{ba}^2 \\mathbf{I})$$  
这个建模决策至关重要。它意味着偏置 $\\mathbf{b}$ 不是一个需要一次性校准的恒定值，而是一个随时间连续变化的*状态*。因此，它必须与位置、速度和姿态一起，在系统的优化过程中被持续地估计和更新。

### **1.3 连续时间运动学模型（“真值”模型）**

给定*真实*的角速度 $\\boldsymbol{\\omega}(t)$ 和*真实*的运动学加速度 $\\mathbf{a}(t)$（即测量值减去偏置和噪声），描述系统状态随时间演化的连续时间微分方程（即运动学方程）如下 3：

位置：位置的时间导数定义为速度。

$$\\dot{\\mathbf{p}}\_W(t) \= \\mathbf{v}\_W(t)$$  
速度：速度的时间导数是世界坐标系中的总加速度。这等于从 $B$ 系旋转到 $W$ 系的真实运动学加速度 $R\_{WB}(t) \\mathbf{a}(t)$，再加上世界坐标系中的重力加速度 $\\mathbf{g}\_W$。

$$\\dot{\\mathbf{v}}\_W(t) \= R\_{WB}(t) \\mathbf{a}(t) \+ \\mathbf{g}\_W$$  
姿态（旋转）：旋转矩阵 $R\_{WB}(t)$ 的时间导数由其自身乘以角速度 $\\boldsymbol{\\omega}(t)$ 的反对称矩阵 $\\lfloor \\boldsymbol{\\omega}(t) \\rfloor\_\\times$ 给出。

$$\\dot{R}\_{WB}(t) \= R\_{WB}(t) \\lfloor \\boldsymbol{\\omega}(t) \\rfloor\_\\times$$  
其中，$\\lfloor \\boldsymbol{\\omega} \\rfloor\_\\times \= \\begin{pmatrix} 0 & \-\\omega\_z & \\omega\_y \\\\ \\omega\_z & 0 & \-\\omega\_x \\\\ \-\\omega\_y & \\omega\_x & 0 \\end{pmatrix}$ 是 $3 \\times 3$ 的反对称矩阵。

### **表1：关键变量与符号定义**

为了确保后续公式推导的清晰性，下表定义了本报告中使用的核心数学符号。

| 符号 | 定义 | 类型 | 坐标系 |
| :---- | :---- | :---- | :---- |
| $W$ | 世界坐标系（惯性系） | 参考系 | N/A |
| $B$ | IMU/机体坐标系（非惯性系） | 参考系 | N/A |
| $L$ | LiDAR坐标系 | 参考系 | N/A |
| $R\_{WB}$ | 从 $B$ 系到 $W$ 系的旋转矩阵 | $SO(3)$ | N/A |
| $\\mathbf{p}\_W$ | $B$ 系在 $W$ 系中的位置 | $\\mathbb{R}^3$ | $W$ |
| $\\mathbf{v}\_W$ | $B$ 系在 $W$ 系中的速度 | $\\mathbb{R}^3$ | $W$ |
| $\\mathbf{g}\_W$ | $W$ 系中的重力向量 | $\\mathbb{R}^3$ | $W$ |
| $\\tilde{\\boldsymbol{\\omega}}$ | 陀螺仪测量值（原始数据） | $\\mathbb{R}^3$ | $B$ |
| $\\boldsymbol{\\omega}$ | 真实角速度 | $\\mathbb{R}^3$ | $B$ |
| $\\tilde{\\mathbf{a}}$ | 加速度计测量值（原始数据，比力） | $\\mathbb{R}^3$ | $B$ |
| $\\mathbf{a}$ | 真实运动学加速度 | $\\mathbb{R}^3$ | $B$ |
| $\\mathbf{b}\_g, \\mathbf{b}\_a$ | 陀螺仪和加速度计的偏置 | $\\mathbb{R}^3$ | $B$ |
| $\\mathbf{n}\_g, \\mathbf{n}\_a$ | 测量白噪声 | $\\mathbb{R}^3$ | $B$ |
| $\\mathbf{n}\_{wg}, \\mathbf{n}\_{wa}$ | 偏置随机游走噪声 | $\\mathbb{R}^3$ | $B$ |
| $\\Delta t$ | 离散时间间隔 | $\\mathbb{R}$ | N/A |
| $\\Delta R\_{ij}, \\Delta v\_{ij}, \\Delta p\_{ij}$ | 预积分测量值（从 $i$ 到 $j$） | $SO(3), \\mathbb{R}^3, \\mathbb{R}^3$ | $B\_i$ |
| $\\Sigma\_{ij}$ | 预积分测量值的协方差 | $9 \\times 9$ 矩阵 | $B\_i$ |
| $\\mathbf{J}\_{\\Delta}^{\\mathbf{b}}$ | 预积分测量值关于偏置的雅可比 | 矩阵 | $B\_i$ |
| $\\lfloor \\cdot \\rfloor\_\\times$ | 向量到反对称矩阵的算子 | 算子 | N/A |
| $\\text{Exp}(\\cdot)$ | $SO(3)$ 流形上的指数映射（从 $\\mathbb{R}^3$ 到 $SO(3)$） | 映射 | N/A |
| $\\text{Log}(\\cdot)$ | $SO(3)$ 流形上的对数映射（从 $SO(3)$ 到 $\\mathbb{R}^3$） | 映射 | N/A |

## **2\. 计算困境：为什么需要预积分**

基于第1节中建立的运动学方程，最直接的（或称为“朴素的”）方法是通过数值积分（称为IMU机械编排）来递推状态。然而，这种方法在现代基于优化的SLAM框架（如LIO-SAM）中会引发严重的计算效率问题。

### **2.1 朴素方法：直接积分（IMU机械编排）**

给定 $t\_k$ 时刻的状态 $X\_k$ 和IMU测量值 $\\tilde{\\boldsymbol{\\omega}}\_k, \\tilde{\\mathbf{a}}\_k$，我们可以使用简单的欧拉积分（Euler integration）来递推 $t\_{k+1}$ 时刻的状态 $X\_{k+1}$（其中 $\\Delta t \= t\_{k+1} \- t\_k$）8。假设偏置 $\\mathbf{b}$ 在此短间隔内为常数：

$$R\_{k+1} \= R\_k \\cdot \\text{Exp}((\\tilde{\\boldsymbol{\\omega}}\_k \- \\mathbf{b}\_g) \\Delta t)$$  
$$\\mathbf{v}\_{k+1} \= \\mathbf{v}\_k \+ \\mathbf{g}\_W \\Delta t \+ R\_k (\\tilde{\\mathbf{a}}\_k \- \\mathbf{b}\_a) \\Delta t$$  
$$\\mathbf{p}\_{k+1} \= \\mathbf{p}\_k \+ \\mathbf{v}\_k \\Delta t \+ \\frac{1}{2} \\mathbf{g}\_W \\Delta t^2 \+ \\frac{1}{2} R\_k (\\tilde{\\mathbf{a}}\_k \- \\mathbf{b}\_a) \\Delta t^2$$  
这种方法在扩展卡尔曼滤波器（EKF）等滤波方法中被广泛用作“预测”步骤 10。

### **2.2 基于优化方法的问题**

像LIO-SAM 11 和 VINS-Mono 13 这样的现代SLAM系统是*基于优化*的，它们通常使用因子图 15 对*过去一段时间内*的所有状态进行非线性最小二乘估计，以获得最大后验概率（MAP）解 14。

这里出现了一个频率不匹配的严重问题：

* **IMU频率（高）**：通常为 200 Hz 到 1000 Hz 17。  
* **LiDAR/相机频率（低）**：通常为 10 Hz 到 30 Hz 17。

这意味着在两个连续的LiDAR关键帧（即因子图中的两个状态节点 $X\_i$ 和 $X\_j$）之间，可能存在数百个IMU测量数据 14。如果使用朴素积分法，状态 $X\_j$ 将成为 $X\_i$ 以及 $i$ 和 $j$ 之间所有IMU测量值 $\\mathbf{u}\_k$ 的函数：

$$X\_j \= f(X\_i, \\mathbf{u}\_i, \\mathbf{u}\_{i+1}, \\dots, \\mathbf{u}\_{j-1})$$  
这会在因子图中 $X\_i$ 和 $X\_j$ 之间创建一条长长的“IMU积分链”。

### **2.3 计算“爆炸”**

基于优化的SLAM（例如使用GTSAM 15）的核心思想是*迭代*地调整图中*所有*变量（包括*过去*的状态）的估计值，以最小化全局误差 14。

**这就是问题的核心**：假设优化器在某次迭代中，为了满足LiDAR约束或回环约束，决定更新（或“重新线性化”）*过去*的某个状态 $X\_i$ 14。

**灾难性的后果**：由于 $X\_j$ 的计算完全依赖于 $X\_i$ 作为*初始值*， $X\_i$ 的任何微小变动都会使从 $i$ 到 $j$ 的整个IMU积分链完全失效 14。系统唯一的选择是*重新积分*这数百个IMU测量值（$\\mathbf{u}\_i \\dots \\mathbf{u}\_{j-1}$），以计算出新的 $X\_j$ 及其对应的雅可比矩阵。

在每次优化迭代中，对每对关键帧都重复执行数百次的积分计算，这在实时系统中是绝对无法接受的计算负担 14。

这种IMU积分（本质上是因果的、前向的初始值问题）与因子图优化（非因果的、全局的、迭代重线性化）之间的根本性冲突，催生了IMU预积分理论。

**解决方案**：我们必须找到一种方法，将 $i$ 和 $j$ 之间的数百个IMU测量值“预先”压缩成一个*单独*的、*相对*的运动约束 $\\Delta \\mathbf{Z}\_{ij}$。这个约束 $\\Delta \\mathbf{Z}\_{ij}$ 必须*独立*于 $X\_i$ 的具体值，从而使优化器在调整 $X\_i$ 时，无需重新计算这个约束 14。这就是IMU预积分的根本目标。

## **3\. 核心理论：IMU预积分的数学推导**

IMU预积分理论的目标是构建一个相对运动测量值 $\\Delta \\mathbf{Z}\_{ij}$，它仅依赖于 $t\_i$ 和 $t\_j$ 之间的IMU原始测量数据，并可用于约束因子图中的状态 $X\_i$ 和 $X\_j$。这个理论包含三个关键部分：预积分的均值、协方差传播和偏置的在线修正。

### **3.1 目标：建立相对运动约束**

我们从第1.3节中的连续时间运动学方程出发，将其改写为连接状态 $X\_i$ 和 $X\_j$ 的相对形式。通过在 $t\_i$ 到 $t\_j$（$\\Delta t\_{ij} \= t\_j \- t\_i$）上积分，可以得到以下“真值”关系 11：

$$R\_j \= R\_i \\cdot \\Delta R\_{ij}$$  
$$\\mathbf{v}\_j \= \\mathbf{v}\_i \+ \\mathbf{g}\_W \\Delta t\_{ij} \+ R\_i \\Delta \\mathbf{v}\_{ij}$$  
$$\\mathbf{p}\_j \= \\mathbf{p}\_i \+ \\mathbf{v}\_i \\Delta t\_{ij} \+ \\frac{1}{2} \\mathbf{g}\_W \\Delta t\_{ij}^2 \+ R\_i \\Delta \\mathbf{p}\_{ij}$$  
这里的 $\\Delta R\_{ij}, \\Delta v\_{ij}, \\Delta p\_{ij}$ 就是我们寻求的预积分项。它们代表了在 $t\_i$ 时刻的机体坐标系 $B\_i$ 下，从 $B\_i$ 到 $B\_j$ 的相对姿态、相对速度和相对位置。我们的任务是推导出*仅*使用 $k \\in \[i, j-1\]$ 区间内的IMU测量值 $\\tilde{\\boldsymbol{\\omega}}\_k, \\tilde{\\mathbf{a}}\_k$ 来计算这些 $\\Delta$ 项的公式。

### **3.2 预积分测量值的推导（均值）**

我们在 $B\_i$ 坐标系中进行积分。假设（暂时）偏置 $\\mathbf{b}$ 在积分区间内是恒定且已知的。

姿态 ($\\Delta R\_{ij}$)  
这是最关键的部分。为了避免欧拉角带来的万向锁（Gimbal Lock）问题 21，姿态积分必须在 $SO(3)$ 流形上进行 23。这通过递归地“右乘”每个微小的旋转增量来实现 5。  
在两个IMU采样 $k$ 和 $k+1$ 之间（间隔为 $\\Delta t\_k$）：

$$\\Delta R\_{k, k+1} \= \\text{Exp}((\\tilde{\\boldsymbol{\\omega}}\_k \- \\mathbf{b}\_g) \\Delta t\_k)$$

总的预积分姿态是所有这些小旋转的连乘：  
$$\\Delta R\_{ij} \= \\prod\_{k=i}^{j-1} \\Delta R\_{k, k+1} \= \\prod\_{k=i}^{j-1} \\text{Exp}((\\tilde{\\boldsymbol{\\omega}}\_k \- \\mathbf{b}\_g) \\Delta t\_k)$$  
速度 ($\\Delta v\_{ij}$)  
这是在 $B\_i$ 坐标系下对加速度（减去偏置）的积分。在 $k+1$ 时刻的预积分速度 $\\Delta \\mathbf{v}\_{i, k+1}$ 是 $k$ 时刻的速度 $\\Delta \\mathbf{v}\_{ik}$ 加上在 $k$ 时刻的姿态 $\\Delta R\_{ik}$ 下积分的加速度：  
$$\\Delta \\mathbf{v}\_{i, k+1} \= \\Delta \\mathbf{v}\_{ik} \+ \\Delta R\_{ik} (\\tilde{\\mathbf{a}}\_k \- \\mathbf{b}\_a) \\Delta t\_k $$总的预积分速度是所有这些增量的累加：$$ \\Delta \\mathbf{v}\_{ij} \= \\sum\_{k=i}^{j-1} \\left( \\Delta R\_{ik} (\\tilde{\\mathbf{a}}\_k \- \\mathbf{b}\_a) \\Delta t\_k \\right)$$  
位置 ($\\Delta p\_{ij}$)  
这是对预积分速度 $\\Delta \\mathbf{v}$ 的积分（即对加速度的二重积分）：  
$$\\Delta \\mathbf{p}\_{i, k+1} \= \\Delta \\mathbf{p}\_{ik} \+ \\Delta \\mathbf{v}\_{ik} \\Delta t\_k \+ \\frac{1}{2} \\Delta R\_{ik} (\\tilde{\\mathbf{a}}\_k \- \\mathbf{b}\_a) \\Delta t\_k^2 $$总的预积分位置是：$$ \\Delta \\mathbf{p}\_{ij} \= \\sum\_{k=i}^{j-1} \\left( \\Delta \\mathbf{v}\_{ik} \\Delta t\_k \+ \\frac{1}{2} \\Delta R\_{ik} (\\tilde{\\mathbf{a}}\_k \- \\mathbf{b}\_a) \\Delta t\_k^2 \\right)$$

### **3.3 不确定性（协方差）的传播**

预积分测量 $\\Delta \\mathbf{Z}\_{ij} \=$ （其中 $\\Delta \\theta$ 是 $\\Delta R$ 在流形切空间上的3D表示）本身是随机变量，因为它们是由带噪声的测量值 $\\tilde{\\boldsymbol{\\omega}}, \\tilde{\\mathbf{a}}$ 和带噪声的偏置 $\\mathbf{b}$ 积分而来的。

我们必须在积分过程中传播这些不确定性（即第1.2节中定义的 $\\mathbf{n}\_g, \\mathbf{n}\_a, \\mathbf{n}\_{wg}, \\mathbf{n}\_{wa}$），以获得最终预积分测量 $\\Delta \\mathbf{Z}\_{ij}$ 的协方差矩阵 $\\Sigma\_{ij}$。这个 $\\Sigma\_{ij}$ 将在优化中用作信息矩阵（$\\Sigma\_{ij}^{-1}$），来“加权”IMU因子 25。

这是一个标准的线性误差传播问题。我们将9D的预积分状态 $\\zeta \=$ 的协方差 $\\Sigma\_k$ 递归地传播到 $\\Sigma\_{k+1}$。根据 26 和 26，离散时间的协方差更新方程为：

$$\\Sigma\_{k+1} \= \\mathbf{A}\_k \\Sigma\_k \\mathbf{A}\_k^T \+ \\mathbf{B}\_k \\Sigma\_m \\mathbf{B}\_k^T$$

* $\\mathbf{A}\_k \= \\frac{\\partial \\zeta\_{k+1}}{\\partial \\zeta\_k}$ 是状态转移雅可比矩阵。  
* $\\mathbf{B}\_k$ 是 $\\zeta\_{k+1}$ 相对于测量噪声和偏置噪声的雅可比矩阵。  
* $\\Sigma\_m$ 是测量噪声和偏置随机游走噪声的协方差矩阵。

根据 26，状态转移雅可比矩阵 $\\mathbf{A}\_k$ 可以近似为（以 $\\zeta \=$ 的顺序为例）：

$$\\mathbf{A}\_k \\approx \\begin{pmatrix} \\mathbf{I} \- \\lfloor (\\tilde{\\boldsymbol{\\omega}}\_k \- \\mathbf{b}\_g) \\Delta t \\rfloor\_\\times & \\mathbf{0} & \\mathbf{0} \\\\ \\frac{1}{2} \\Delta R\_{ik} \\lfloor \-(\\tilde{\\mathbf{a}}\_k \- \\mathbf{b}\_a) \\Delta t^2 \\rfloor\_\\times & \\mathbf{I} & \\mathbf{I}\\Delta t \\\\ \\Delta R\_{ik} \\lfloor \-(\\tilde{\\mathbf{a}}\_k \- \\mathbf{b}\_a) \\Delta t \\rfloor\_\\times & \\mathbf{0} & \\mathbf{I} \\end{pmatrix}$$  
通过在 $k \\in \[i, j-1\]$ 区间内不断迭代此公式，我们最终得到总的预积分协方差 $\\Sigma\_{ij}$。

### **3.4 偏置的在线修正（雅可比推导）**

第3.2节中的推导是基于一个*错误*的假设：偏置 $\\mathbf{b}$ 是已知的。在实际系统中，我们只使用一个*估计值* $\\tilde{\\mathbf{b}}$ 来计算预积分项，记为 $\\Delta \\tilde{\\mathbf{Z}}(\\tilde{\\mathbf{b}})$。

在优化过程中，GTSAM会找到一个*更优*的偏置估计 $\\mathbf{b} \= \\tilde{\\mathbf{b}} \+ \\delta \\mathbf{b}$。此时，我们*绝不能*为了这个小小的 $\\delta \\mathbf{b}$ 而重新积分（见第2.3节）。

解决方案是使用一阶泰勒展开，*线性地*修正预积分测量值，以反映偏置 $\\delta \\mathbf{b}$ 的变化 8：

$$\\Delta \\mathbf{Z}(\\mathbf{b}) \\approx \\Delta \\tilde{\\mathbf{Z}}(\\tilde{\\mathbf{b}}) \+ \\mathbf{J}\_{\\Delta \\mathbf{Z}}^{\\mathbf{b}} \\delta \\mathbf{b}$$  
这里的 $\\mathbf{J}\_{\\Delta \\mathbf{Z}}^{\\mathbf{b}} \= \\frac{\\partial \\Delta \\mathbf{Z}}{\\partial \\mathbf{b}}$ 是预积分测量值关于偏置的雅可比矩阵。这个雅可比矩阵也必须在预积分过程中递归计算并存储起来。

根据 28（VINS-Mono补充材料）中的详细推导，这些雅可比矩阵的最终形式（表示为离散求和）为：

**加速度计偏置 ($\\mathbf{b}\_a$) 的影响**（这比较简单，因为它不影响旋转）：

$$\\frac{\\partial \\Delta \\mathbf{p}\_{ij}}{\\partial \\mathbf{b}\_a} \= \- \\sum\_{k=i}^{j-1} \\left( \\frac{1}{2} \\Delta R\_{ik} \\Delta t\_k^2 \\right)$$  
$$\\frac{\\partial \\Delta \\mathbf{v}\_{ij}}{\\partial \\mathbf{b}\_a} \= \- \\sum\_{k=i}^{j-1} \\left( \\Delta R\_{ik} \\Delta t\_k \\right)$$  
**陀螺仪偏置 ($\\mathbf{b}\_g$) 的影响**（这非常复杂，因为 $\\mathbf{b}\_g$ 的变化 $\\delta \\mathbf{b}\_g$ 会影响 $\\Delta R\_{ik}$，而 $\\Delta R\_{ik}$ 又会影响 $\\Delta \\mathbf{p}$ 和 $\\Delta \\mathbf{v}$ 的积分）：

$$\\frac{\\partial \\Delta R\_{ij}}{\\partial \\mathbf{b}\_g} \\approx \- \\sum\_{k=i}^{j-1} \\left \\quad (\\text{其中 } \\mathbf{J}\_r^k \\text{ 是SO(3)的右雅可比})$$  
$$\\frac{\\partial \\Delta \\mathbf{p}\_{ij}}{\\partial \\mathbf{b}\_g} \\approx \\sum\_{k=i}^{j-1} \\left( \\frac{\\partial \\Delta \\mathbf{v}\_{ik}}{\\partial \\mathbf{b}\_g} \\Delta t\_k \- \\frac{1}{2} \\Delta R\_{ik} \\lfloor (\\tilde{\\mathbf{a}}\_k \- \\mathbf{b}\_a) \\rfloor\_\\times \\frac{\\partial \\Delta R\_{ik}}{\\partial \\mathbf{b}\_g} \\Delta t\_k^2 \\right)$$  
$$\\frac{\\partial \\Delta \\mathbf{v}\_{ij}}{\\partial \\mathbf{b}\_g} \\approx \- \\sum\_{k=i}^{j-1} \\left( \\Delta R\_{ik} \\lfloor (\\tilde{\\mathbf{a}}\_k \- \\mathbf{b}\_a) \\rfloor\_\\times \\frac{\\partial \\Delta R\_{ik}}{\\partial \\mathbf{b}\_g} \\Delta t\_k \\right)$$  
综上所述，IMU预积分理论的完整产物是一个“数据包”，它在 $t\_i$ 和 $t\_j$ 之间生成并传递给优化器。这个数据包包含三个核心组件：  
$\\{\\Delta \\tilde{\\mathbf{Z}}(\\tilde{\\mathbf{b}}), \\Sigma\_{ij}, \\mathbf{J}\_{\\Delta \\mathbf{Z}}^{\\mathbf{b}}\\}$  
（均值、协方差、偏置雅可比）  
缺少其中任何一个，基于优化的VIO/LIO系统都无法在计算效率和统计最优性之间取得平衡。

## **4\. LIO-SAM应用（一）：前端LiDAR畸变校正（De-skewing）**

LIO-SAM对IMU数据的*第一个*应用是在前端处理中，用于校正LiDAR点云的运动畸变。

### **4.1 LiDAR的“卷帘快门”问题**

机械式旋转LiDAR（如Velodyne）在扫描时会遇到类似相机“卷帘快门”的问题 29。一次360度的扫描需要时间（例如 10 Hz 的LiDAR需要 0.1 秒）2。

在这 0.1 秒内，点云中的点不是同时采集的。第一个点和最后一个点之间有 0.1 秒的时间差。如果机器人在此期间运动（平移或旋转），采集到的点云将会被“扭曲”或“涂抹”30。例如，一面笔直的墙壁在点云中可能显示为弯曲的 29。这种现象称为运动畸变（motion distortion）。

### **4.2 LIO-SAM的解决方案：逐点运动补偿**

LIO-SAM利用IMU预积分提供的高频位姿估计来解决这个前端问题 2。  
这个核心逻辑位于 imageProjection.cpp 节点中 2。  
其目标是：将扫描周期内的*每一个点* $\\mathbf{p}\_k$（在其各自的采集时刻 $t\_k$）全部转换到扫描*开始时刻* $t\_s$ 的LiDAR坐标系 $L\_s$ 中。

该过程（在 deskewPoint() 函数中实现 2）如下：

1. **获取时间戳**：对于LiDAR点云中的每个点 $\\mathbf{p}\_k$，从LiDAR驱动程序中提取其*相对时间戳* $\\Delta t\_k \= t\_k \- t\_s$。这个时间戳信息是必须的 2。  
2. **查询相对变换**：系统需要知道在 $\\Delta t\_k$ 这段微小时间内，机器人（即LiDAR坐标系）发生的相对运动变换 $T\_{L\_k L\_s}$（从 $t\_k$ 时刻的坐标系 $L\_k$ 到 $t\_s$ 时刻的坐标系 $L\_s$）。  
3. **高频位姿来源**：这个相对变换 $T\_{L\_k L\_s}$ 来自哪里？它来自 imuPreintegration.cpp 节点（即LIO-SAM的第一个因子图）输出的*高频*里程计 2。  
4. **插值**：由于 $\\Delta t\_k$ 不可能精确匹配IMU里程计的时间戳，系统会在两个最近的IMU位姿之间进行*插值*（例如线性插值或球面线性插值Slerp）32，以获得 $t\_k$ 时刻的精确相对位姿 $T\_{L\_k L\_s}$。  
5. **畸变校正**：应用变换，将点 $\\mathbf{p}\_k$ 从其自身坐标系 $L\_k$ 转换回开始时刻的坐标系 $L\_s$：  
   $$\\mathbf{p}\_s \= T\_{L\_s L\_k} \\mathbf{p}\_k$$  
   （其中 $\\mathbf{p}\_k$ 是在 $L\_k$ 系下测量的坐标，$\\mathbf{p}\_s$ 是校正后在 $L\_s$ 系下的坐标）。  
6. **输出**：一个“无畸变”（de-skewed）的点云 36，其中所有点都被投影到了同一个坐标系 $t\_s$ 下，仿佛它们是瞬时采集的。

LIO-SAM的LiDAR里程计继承自LOAM 37，其核心是提取点云中的几何特征（边缘线和平面）37。运动畸变会彻底破坏这些几何特征（例如，直线变曲线）29。如果特征提取算法（输入的是畸变点云）无法找到可靠的线和面，后续的“点到线”和“点到面”匹配将失败。因此，imageProjection.cpp 中的IMU畸变校正 31 是一个*必须*的预处理步骤，它“修复”了点云的几何形状，为后续的LiDAR里程计（scan-to-map匹配）39 奠定了基础。

LIO-SAM的系统架构 2 精妙地解决了畸变校正（需要低延迟）和全局优化（高延迟）之间的冲突。它维护了*两个*因子图：

1. **快速图 (imuPreintegration.cpp)**：一个小的、短期的滑动窗口图。它只优化近期的IMU和LiDAR里程计因子，其目的是提供*实时*的里程计输出 2 和*实时*的偏置估计 33。  
2. **慢速图 (mapOptimization.cpp)**：一个大的、持久的全局图。它负责构建地图，并融合GPS 40 和回环检测 12 等低频、高成本的因子。

前端的畸变校正节点 (imageProjection.cpp) 从“快速图”中读取其所需的低延迟运动估计，从而实现了高频畸变校正，而不必等待全局优化的（较慢的）结果。

## **5\. LIO-SAM应用（二）：后端IMU预积分因子**

在前端完成畸变校正后，IMU预积分数据包被*再次*使用，这一次是作为*约束*（因子）被添加到后端的全局因子图中 12。

### **5.1 IMU因子在LIO-SAM因子图中的角色**

LIO-SAM使用GTSAM（Georgia Tech Smoothing and Mapping）库 2 来管理和优化其后端因子图。因子图由*变量节点*（待估计的状态）和*因子节点*（约束变量的测量）组成 15。

* **变量节点 ($X\_i$)**：在LIO-SAM中，每个LiDAR关键帧 $i$ 对应的状态节点 $X\_i$ 包括姿态（$R\_i, \\mathbf{p}\_i$）、速度 $\\mathbf{v}\_i$ 和IMU偏置 $\\mathbf{b}\_i$ 12。  
  $$X\_i \= \\{ R\_i, \\mathbf{p}\_i, \\mathbf{v}\_i, \\mathbf{b}\_i \\}$$  
* **因子节点**：LIO-SAM主要使用四种因子：(1) IMU预积分因子, (2) LiDAR里程计因子, (3) GPS因子, (4) 回环因子 12。

GTSAM中的 ImuFactor 44 是一个多变量因子，它连接并约束两个相邻的关键帧状态 $X\_i$ 和 $X\_j$（具体地说是 $R\_i, \\mathbf{p}\_i, \\mathbf{v}\_i, \\mathbf{b}\_i$ 和 $R\_j, \\mathbf{p}\_j, \\mathbf{v}\_j, \\mathbf{b}\_j$）45。

### **5.2 LIO-SAM的IMU残差函数推导**

优化（即求解因子图）的目标是最小化所有因子定义的残差（error）的总和。IMU因子的残差 $r\_{IMU}$ 衡量的是 (A) *根据当前估计的状态 $X\_i, X\_j$ 预测的相对运动* 与 (B) *IMU预积分测量的相对运动* 之间的差异。

A) 预测的相对运动（来自状态变量）  
根据第3.1节的运动学方程 11，我们可以从状态变量 $X\_i, X\_j$ 中“预测”出预积分项应该是什么样子：

$$\\hat{\\Delta R}\_{ij} (R\_i, R\_j) \= R\_i^T R\_j$$  
$$\\hat{\\Delta v}\_{ij} (X\_i, X\_j) \= R\_i^T ( \\mathbf{v}\_j \- \\mathbf{v}\_i \- \\mathbf{g}\_W \\Delta t\_{ij} )$$  
$$\\hat{\\Delta p}\_{ij} (X\_i, X\_j) \= R\_i^T ( \\mathbf{p}\_j \- \\mathbf{p}\_i \- \\mathbf{v}\_i \\Delta t\_{ij} \- \\frac{1}{2} \\mathbf{g}\_W \\Delta t\_{ij}^2 )$$  
B) 测量的相对运动（来自IMU预积分数据包）  
这即是第3节中推导的“数据包”：$\\{\\Delta \\tilde{\\mathbf{Z}}(\\tilde{\\mathbf{b}}), \\Sigma\_{ij}, \\mathbf{J}\_{\\Delta \\mathbf{Z}}^{\\mathbf{b}}\\}$。  
我们必须使用第3.4节的在线修正公式，将存储的测量值 $\\Delta \\tilde{\\mathbf{Z}}(\\tilde{\\mathbf{b}})$（它是用旧的偏置 $\\tilde{\\mathbf{b}}$ 计算的）修正到当前优化迭代中的偏置估计 $\\mathbf{b}\_i$：  
$$\\Delta \\mathbf{Z}'(\\mathbf{b}\_i) \\approx \\Delta \\tilde{\\mathbf{Z}}(\\tilde{\\mathbf{b}}) \+ \\mathbf{J}\_{\\Delta \\mathbf{Z}}^{\\mathbf{b}} (\\mathbf{b}\_i \- \\tilde{\\mathbf{b}})$$  
其中 $(\\mathbf{b}\_i \- \\tilde{\\mathbf{b}})$ 是偏置的修正量 $\\delta \\mathbf{b}$。

C) IMU残差向量 $r\_{IMU}$  
残差即是“预测值”与“（修正后的）测量值”之差：  
$$\\mathbf{r}\_{ij} \= \\begin{bmatrix} \\mathbf{r}\_R \\\\ \\mathbf{r}\_p \\\\ \\mathbf{r}\_v \\end{bmatrix} \= \\begin{bmatrix} \\text{Log} \\left( (\\Delta \\mathbf{Z}\_R'(\\mathbf{b}\_i))^T \\cdot \\hat{\\Delta R}\_{ij} (R\_i, R\_j) \\right)^\\lor \\\\ \\hat{\\Delta p}\_{ij} (X\_i, X\_j) \- \\Delta \\mathbf{Z}\_p'(\\mathbf{b}\_i) \\\\ \\hat{\\Delta v}\_{ij} (X\_i, X\_j) \- \\Delta \\mathbf{Z}\_v'(\\mathbf{b}\_i) \\end{bmatrix}$$  
注意：姿态残差 $\\mathbf{r}\_R$ 必须在 $SO(3)$ 流形上计算（通过 $\\text{Log}$ 映射将其转换为切空间中的 $\\mathbb{R}^3$ 向量）5。

D) 偏置残差 $\\mathbf{r}\_b$  
还有一个残差用于约束偏置的随机游走（见1.2节），它惩罚两个相邻状态之间偏置的剧烈变化 4：

$$\\mathbf{r}\_b \= \\mathbf{b}\_j \- \\mathbf{b}\_i$$  
E) 最终代价函数  
优化器（如GTSAM）的目标是求解最小化所有这些残差的马氏距离（Mahalanobis distance）的平方和：  
$$\\min\_{X} \\sum\_{ij} \\left( \\| \\mathbf{r}\_{ij} \\|^2\_{\\Sigma\_{ij}^{-1}} \+ \\| \\mathbf{r}\_b \\|^2\_{\\Sigma\_b^{-1}} \\right)$$  
其中 $\\Sigma\_{ij}$ 是第3.3节推导的 $9 \\times 9$ 预积分协方差，$\\Sigma\_b$ 是偏置随机游走的协方差。

### **表2：IMU预积分残差对比（VINS-Mono vs. LIO-SAM）**

LIO-SAM和VINS-Mono是IMU预积分理论的两个最著名的开源实现。虽然它们都基于28中提出的相同核心思想，但在公式细节上略有差异 38。

| 对比项 | VINS-Mono (基于 ) | LIO-SAM (基于 ) |
| :---- | :---- | :---- |
| **残差分量** | 姿态, 位置, 速度, 加速度计偏置, 陀螺仪偏置 (15维) | 姿态, 位置, 速度 (9维) \+ 单独的偏置因子 (6维) |
| **位置残差 ($r\_p$)** | $R\_i^T(\\mathbf{p}\_j \- \\mathbf{p}\_i \- \\mathbf{v}\_i \\Delta t \+ \\frac{1}{2} \\mathbf{g}\_W \\Delta t^2) \- \\Delta \\tilde{\\mathbf{p}}\_{ij}$ | $R\_i^T(\\mathbf{p}\_j \- \\mathbf{p}\_i \- \\mathbf{v}\_i \\Delta t \- \\frac{1}{2} \\mathbf{g}\_W \\Delta t^2) \- \\Delta \\mathbf{Z}\_p'(\\mathbf{b}\_i)$ |
| **速度残差 ($r\_v$)** | $R\_i^T(\\mathbf{v}\_j \- \\mathbf{v}\_i \+ \\mathbf{g}\_W \\Delta t) \- \\Delta \\tilde{\\mathbf{v}}\_{ij}$ | $R\_i^T(\\mathbf{v}\_j \- \\mathbf{v}\_i \- \\mathbf{g}\_W \\Delta t) \- \\Delta \\mathbf{Z}\_v'(\\mathbf{b}\_i)$ |
| **姿态残差 ($r\_R$)** | $2 \\left\_{xyz}$ (四元数) | $\\text{Log} \\left( (\\Delta \\mathbf{Z}\_R'(\\mathbf{b}\_i))^T \\cdot (R\_i^T R\_j) \\right)^\\lor$ ($SO(3)$) |
| **偏置修正** | 在残差函数内部进行修正 47 | 在计算残差*之前*修正测量值 28 |
| **重力向量 $\\mathbf{g}\_W$** | 作为一个变量在线估计 | 假设为固定值 (例如 $\[0,0,-g\]$)，不参与优化 |

*注：VINS-Mono残差公式中的重力符号与LIO-SAM相反，这可能源于其对 $\\Delta \\tilde{\\mathbf{p}}\_{ij}$ 和 $\\Delta \\tilde{\\mathbf{v}}\_{ij}$ 的定义方式不同，但最终的物理约束是等价的。*

LIO-SAM的因子图中包含多种因子（LiDAR, IMU, GPS, 回环）12。通过分析这些因子可以发现：

* LiDAR里程计因子 39 约束 $\\text{Pose}\_i$ 和 $\\text{Pose}\_j$。  
* GPS因子 40 约束绝对位置 $\\mathbf{p}\_k$。  
* 回环因子 12 约束 $\\text{Pose}\_i$ 和 $\\text{Pose}\_m$。

在所有这些因子中，*只有* ImuFactor 提供了对状态变量*速度* $\\mathbf{v}\_k$ 和*偏置* $\\mathbf{b}\_k$ 的约束。这意味着系统对速度和偏置的估计*完全*依赖于IMU预积分因子的正确性。如果IMU预积分理论实现错误或IMU数据质量差，速度和偏置的估计将会发散。更糟糕的是，由于偏置估计 $\\mathbf{b}$ 会被反馈回前端的畸变校正模块（见第4节），一个发散的偏置将导致畸变校正失败，进而导致LiDAR里程计匹配错误，最终造成整个系统崩溃 33。

## **6\. LIO-SAM实现的局限性与关键细节**

虽然LIO-SAM是一个非常成功的系统，但对其开源代码和相关社区报告的分析揭示了一些理论与实现之间的差距，这些差距对于深刻理解其IMU系统至关重要。

### **6.1 实现缺陷（一）：万向锁（Gimbal Lock）问题**

第3.2节强调了在 $SO(3)$ 流形上积分姿态以*避免*万向锁（Gimbal Lock）的重要性 22。GTSAM库本身在内部正确地处理了 $SO(3)$ 流形 5。

然而，多个报告指出LIO-SAM的实现版本“由于使用欧拉角表示而存在万向锁问题” 21。

一份详细的GitHub问题报告（Issue \#243）定位了这个缺陷 50。问题出在LIO-SAM的“胶水代码”中（mapOptimization.cpp），即在PCL数据类型和GTSAM数据类型之间转换的代码。该代码使用了 pcl::getTranslationAndEulerAngles 函数来从变换矩阵中提取欧拉角。

报告 50 指出，该函数计算俯仰角（pitch）的公式为：  
$pitch \= \\text{asin}(-t(2, 0))$  
其中 $t(2, 0)$ 是变换矩阵的对应元素。  
当机器人姿态接近垂直（俯仰角为 $\\pm 90$ 度）时，即发生万向锁的奇异点 22，$t(2, 0)$ 的值会接近 $\\pm 1$。由于浮点数的不精确性，该值可能轻微超出 $\[-1, 1\]$ 的范围，导致 asin() 函数返回 NaN（Not a Number）。这个 NaN 值随后被传入GTSAM优化器，导致 imuPreintegration 节点崩溃 50。

这是一个严重的实现缺陷，它在系统的数据流中重新引入了理论上本应被 $SO(3)$ 预积分所解决的奇异点问题。

### **6.2 实现缺陷（二）：“杆臂”补偿Bug**

LIO-SAM的系统架构要求所有测量数据都在LiDAR坐标系 $L$ 中进行处理 2。这意味着IMU的测量值（在 $B$ 系中）必须首先被转换到 $L$ 系。

一份关键的GitHub问题报告（Issue \#204）52 指出，LIO-SAM的原始实现中存在一个严重的物理错误：代码*只*应用了 $B$ 和 $L$ 之间的旋转外参 $R\_{LB}$，而*忽略*了它们之间的平移外参 $\\mathbf{p}\_{BL}$（即“杆臂”）。

正确的刚体运动学表明， $L$ 系的加速度 $a\_L$ 不仅仅是 $R\_{LB} a\_B$。它还必须包括由IMU绕 $L$ 系旋转而产生的向心加速度和切向加速度项：

$$a\_L \= R\_{LB} a\_B \+ \\lfloor \\dot{\\boldsymbol{\\omega}}\_B \\rfloor\_\\times \\mathbf{p}\_{BL} \+ \\lfloor \\boldsymbol{\\omega}\_B \\rfloor\_\\times (\\lfloor \\boldsymbol{\\omega}\_B \\rfloor\_\\times \\mathbf{p}\_{BL})$$  
如报告 52 所述，这个Bug的后果是，即使用户的机器人在*原地绕LiDAR中心旋转*（$L$ 系没有平移加速度），IMU测量到的加速度（在 $B$ 系）在被错误地转换后，也会在 $L$ 系中产生一个*虚假*的平移加速度。这违反了系统赖以生存的运动学模型。

### **6.3 系统初始化：对9轴IMU的依赖**

LIO-SAM的官方文档明确指出，该系统（与其前身LOAM一样）*要求*使用9轴IMU（即包含磁力计）2。

原因在于系统的初始化方式。与VINS-Mono不同，LIO-SAM不执行复杂的在线初始化程序来动态求解重力向量的方向 47。

相反，LIO-SAM采取了一种“捷径”：

1. 它*信任*9轴IMU内部滤波器输出的*绝对*横滚角（Roll）和俯仰角（Pitch）（通过加速度计感知重力）。  
2. 它*信任*IMU输出的*绝对*偏航角（Yaw）（通过磁力计感知地磁北）。

系统使用这些由IMU硬件直接提供的角度来初始化系统的初始姿态 2。这种方法简化了算法，但代价是系统性能依赖于IMU硬件滤波器的质量，并且极易受到环境磁场（例如室内的钢筋混凝土）的干扰。LIO-SAM文档 2 坦承：“理论上，像VINS-Mono那样的初始化程序将使LIO-SAM能够使用6轴IMU。” 这表明LIO-SAM在设计上做出了取舍，将复杂的初始化问题“外包”给了IMU硬件。

## **7\. 总结：LIO-SAM中IMU数据的完整流程**

综合以上所有理论和分析，IMU数据在LIO-SAM系统中的完整生命周期是一个精巧的闭环，跨越了所有主要节点：

* **步骤 1：输入与转换 (前端, imageProjection.cpp)**  
  * IMU原始测量值 $\\tilde{\\boldsymbol{\\omega}}\_B, \\tilde{\\mathbf{a}}\_B$ 到达 imuHandler() 2。  
  * 数据通过外参 extrinsicRot 2 和（修正后的）杆臂补偿 52，从 $B$ 系转换到 $L$ 系。  
* **步骤 2：预积分与快速里程计 (图 1, imuPreintegration.cpp)**  
  * 转换后的测量值被送入此节点 2。  
  * 执行第3节的预积分理论：计算均值 $\\Delta \\tilde{\\mathbf{Z}}$、协方差 $\\Sigma\_{ij}$ 和偏置雅可比 $\\mathbf{J}\_{\\Delta \\mathbf{Z}}^{\\mathbf{b}}$。  
  * 一个高频的、小型的滑动窗口因子图被维护和优化 2。  
  * **输出 (A)**: 一个高频的（例如IMU频率）里程计位姿轨迹 2。  
  * **输出 (B)**: 一个*实时*的IMU偏置估计 $\\mathbf{b}\_{realtime}$ 33。  
* **步骤 3：畸变校正 (前端, imageProjection.cpp)**  
  * 一个新的原始LiDAR扫描点云到达 2。  
  * deskewPoint() 函数 2 查询 **步骤 2A** 中生成的高频里程计轨迹。  
  * 通过插值 32，点云中的*每个点* $\\mathbf{p}\_k$ 都被转换回扫描开始时刻 $t\_s$ 的坐标系（见4.2节）。  
  * **输出**: 一个无畸变的、几何一致的点云，发送给特征提取模块。  
* **步骤 4：全局优化 (图 2, mapOptimization.cpp)**  
  * 无畸变的点云被处理，生成一个*LiDAR里程计因子*。  
  * 该因子与 **步骤 2** 中生成的IMU预积分“数据包”($\\{\\Delta \\tilde{\\mathbf{Z}}, \\Sigma, \\mathbf{J}\\}$) 一起被添加到全局因子图中 2。  
  * GTSAM优化器求解这个包含所有因子（IMU, LiDAR, GPS, 回环）的大型非线性最小二乘问题 12，使用第5.2节推导的IMU残差函数。  
  * **输出 (C)**: 一个全局一致、漂移被修正的优化轨迹。  
  * **输出 (D)**: 一个*全局优化后*的、更准确的IMU偏置估计 $\\mathbf{b}\_{optimal}$ 33。  
* **步骤 5：偏置反馈闭环**  
  * **步骤 4D** 中生成的 $\\mathbf{b}\_{optimal}$ 被*反馈*回 imuPreintegration.cpp 节点（图 1）33。  
  * 这个 $\\mathbf{b}\_{optimal}$ 成为**步骤 2** 中*未来*预积分计算的*新*的线性化点（即 $\\tilde{\\mathbf{b}}$）。

这个反馈循环是LIO-SAM架构的核心：它允许“快速图”（图1）为前端提供低延迟的畸变校正，同时又允许“慢速图”（图2）利用全局信息（如GPS和回环）来校正“快速图”的偏置漂移，从而防止整个系统发散。

#### **Works cited**

1. Using Inertial Sensors for Position and Orientation Estimation, accessed November 13, 2025, [https://cw.fel.cvut.cz/b212/\_media/courses/b0b37nsi/tutorials/1704.06053.pdf](https://cw.fel.cvut.cz/b212/_media/courses/b0b37nsi/tutorials/1704.06053.pdf)  
2. LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping \- GitHub, accessed November 13, 2025, [https://github.com/TixiaoShan/LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)  
3. Covariance Pre-Integration for Delayed Measurements in Multi-Sensor Fusion \- Universität Klagenfurt, accessed November 13, 2025, [https://www.aau.at/wp-content/uploads/2019/08/Covariance-Pre-Integration-IROS19.pdf](https://www.aau.at/wp-content/uploads/2019/08/Covariance-Pre-Integration-IROS19.pdf)  
4. IMU Propagation Derivations | OpenVINS, accessed November 13, 2025, [https://docs.openvins.com/propagation.html](https://docs.openvins.com/propagation.html)  
5. Continuous Integration over SO(3) for IMU Preintegration \- Robotics, accessed November 13, 2025, [https://www.roboticsproceedings.org/rss17/p078.pdf](https://www.roboticsproceedings.org/rss17/p078.pdf)  
6. IMU Noise Model · ethz-asl/kalibr Wiki \- GitHub, accessed November 13, 2025, [https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)  
7. IMU Fundamentals, Part 3: Stochastic Error Modeling \- Tangram Visions Blog, accessed November 13, 2025, [https://www.tangramvision.com/blog/stochastic-imu-error-modeling](https://www.tangramvision.com/blog/stochastic-imu-error-modeling)  
8. IMU Fundamentals, Part 5: Preintegration Basics \- Tangram Visions Blog, accessed November 13, 2025, [https://www.tangramvision.com/blog/imu-preintegration-basics-part-5-of-5](https://www.tangramvision.com/blog/imu-preintegration-basics-part-5-of-5)  
9. Continuous-Time State Estimation Methods in Robotics: A Survey \- arXiv, accessed November 13, 2025, [https://arxiv.org/html/2411.03951v1](https://arxiv.org/html/2411.03951v1)  
10. Continuous-Time State Estimation Methods in Robotics: A Survey \- ETH Research Collection, accessed November 13, 2025, [https://www.research-collection.ethz.ch/bitstreams/c77e42e8-e9d2-4695-87ff-2dae1d785e8f/download](https://www.research-collection.ethz.ch/bitstreams/c77e42e8-e9d2-4695-87ff-2dae1d785e8f/download)  
11. LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping \- MIT Senseable City Lab, accessed November 13, 2025, [https://senseable.mit.edu/papers/pdf/20201020\_Shan-etal\_LIO-SAM\_IROS.pdf](https://senseable.mit.edu/papers/pdf/20201020_Shan-etal_LIO-SAM_IROS.pdf)  
12. MIT Open Access Articles LIO-SAM: Tightly-coupled ... \- DSpace@MIT, accessed November 13, 2025, [https://dspace.mit.edu/bitstream/handle/1721.1/144041/2007.00258.pdf?sequence=2\&isAllowed=y](https://dspace.mit.edu/bitstream/handle/1721.1/144041/2007.00258.pdf?sequence=2&isAllowed=y)  
13. Monocular Visual-Inertial SLAM: Continuous Preintegration and Reliable Initialization \- PMC, accessed November 13, 2025, [https://pmc.ncbi.nlm.nih.gov/articles/PMC5712814/](https://pmc.ncbi.nlm.nih.gov/articles/PMC5712814/)  
14. IMU Preintegration on Manifold for Efficient Visual-Inertial ... \- Robotics, accessed November 13, 2025, [https://www.roboticsproceedings.org/rss11/p06.pdf](https://www.roboticsproceedings.org/rss11/p06.pdf)  
15. Factor Graphs and GTSAM, accessed November 13, 2025, [https://gtsam.org/tutorials/intro.html](https://gtsam.org/tutorials/intro.html)  
16. On-Manifold Preintegration for Real-Time Visual-Inertial Odometry \- Robotics and Perception Group, accessed November 13, 2025, [https://rpg.ifi.uzh.ch/docs/TRO16\_forster.pdf](https://rpg.ifi.uzh.ch/docs/TRO16_forster.pdf)  
17. Any resources to learn about IMU pre integration? : r/robotics \- Reddit, accessed November 13, 2025, [https://www.reddit.com/r/robotics/comments/17ni2od/any\_resources\_to\_learn\_about\_imu\_pre\_integration/](https://www.reddit.com/r/robotics/comments/17ni2od/any_resources_to_learn_about_imu_pre_integration/)  
18. accessed November 13, 2025, [https://www.reddit.com/r/robotics/comments/17ni2od/any\_resources\_to\_learn\_about\_imu\_pre\_integration/\#:\~:text=Preintegration%20is%20a%20way%20to,the%20nonlinear%20least%20squares%20optimization.](https://www.reddit.com/r/robotics/comments/17ni2od/any_resources_to_learn_about_imu_pre_integration/#:~:text=Preintegration%20is%20a%20way%20to,the%20nonlinear%20least%20squares%20optimization.)  
19. \[1512.02363\] On-Manifold Preintegration for Real-Time Visual-Inertial Odometry \- arXiv, accessed November 13, 2025, [https://arxiv.org/abs/1512.02363](https://arxiv.org/abs/1512.02363)  
20. Extended Preintegration for Relative State Estimation of Leader-Follower Platform \- arXiv, accessed November 13, 2025, [https://arxiv.org/abs/2308.07723](https://arxiv.org/abs/2308.07723)  
21. VOX-LIO: An Effective and Robust LiDAR-Inertial Odometry System Based on Surfel Voxels, accessed November 13, 2025, [https://www.mdpi.com/2072-4292/17/13/2214](https://www.mdpi.com/2072-4292/17/13/2214)  
22. Gimbal lock \- Wikipedia, accessed November 13, 2025, [https://en.wikipedia.org/wiki/Gimbal\_lock](https://en.wikipedia.org/wiki/Gimbal_lock)  
23. On-Manifold Preintegration for Real-Time Visual-Inertial Odometry \- Frank Dellaert, accessed November 13, 2025, [https://dellaert.github.io/files/Forster16tro.pdf](https://dellaert.github.io/files/Forster16tro.pdf)  
24. The Unified Mathematical Framework for IMU Preintegration in Inertial-Aided Navigation System \- arXiv, accessed November 13, 2025, [https://arxiv.org/pdf/2111.09100](https://arxiv.org/pdf/2111.09100)  
25. A Study on Graph Optimization Method for GNSS/IMU Integrated Navigation System Based on Virtual Constraints \- PMC \- NIH, accessed November 13, 2025, [https://pmc.ncbi.nlm.nih.gov/articles/PMC11244597/](https://pmc.ncbi.nlm.nih.gov/articles/PMC11244597/)  
26. The New IMU Factor \- GitHub, accessed November 13, 2025, [https://raw.githubusercontent.com/borglab/gtsam/develop/doc/ImuFactor.pdf](https://raw.githubusercontent.com/borglab/gtsam/develop/doc/ImuFactor.pdf)  
27. Real-Time Truly-Coupled Lidar-Inertial Motion Correction and Spatiotemporal Dynamic Object Detection \- arXiv, accessed November 13, 2025, [https://arxiv.org/html/2410.05152v1](https://arxiv.org/html/2410.05152v1)  
28. Supplementary Material to: IMU Preintegration on Manifold for ..., accessed November 13, 2025, [https://rpg.ifi.uzh.ch/docs/RSS15\_Forster\_Supplementary.pdf](https://rpg.ifi.uzh.ch/docs/RSS15_Forster_Supplementary.pdf)  
29. LOAM: Lidar Odometry and Mapping in Real-time \- Carnegie Mellon University Robotics Institute, accessed November 13, 2025, [https://www.ri.cmu.edu/pub\_files/2014/7/Ji\_LidarMapping\_RSS2014\_v8.pdf](https://www.ri.cmu.edu/pub_files/2014/7/Ji_LidarMapping_RSS2014_v8.pdf)  
30. A High Accuracy LiDAR-Inertial Odometry Using Undistorted Sectional Point \- IEEE Xplore, accessed November 13, 2025, [https://ieeexplore.ieee.org/iel7/6287639/10005208/10363186.pdf](https://ieeexplore.ieee.org/iel7/6287639/10005208/10363186.pdf)  
31. LIO-SAM: Tightly-coupled Lidar Inertial Odometry via ... \- IEEE Xplore, accessed November 13, 2025, [https://ieeexplore.ieee.org/iel7/9340668/9340635/09341176.pdf](https://ieeexplore.ieee.org/iel7/9340668/9340635/09341176.pdf)  
32. 3D LiDAR Point Cloud Registration Based on IMU Preintegration in ..., accessed November 13, 2025, [https://pmc.ncbi.nlm.nih.gov/articles/PMC10098805/](https://pmc.ncbi.nlm.nih.gov/articles/PMC10098805/)  
33. LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping \- arXiv, accessed November 13, 2025, [https://arxiv.org/abs/2007.00258](https://arxiv.org/abs/2007.00258)  
34. LIO-SAM \- Siyuan Ren \- GitLab, accessed November 13, 2025, [https://gitlab.uni-hannover.de/esistoiamk/LIO-SAM](https://gitlab.uni-hannover.de/esistoiamk/LIO-SAM)  
35. 3D LiDAR Point Cloud Registration Based on IMU Preintegration in Coal Mine Roadways, accessed November 13, 2025, [https://www.mdpi.com/1424-8220/23/7/3473](https://www.mdpi.com/1424-8220/23/7/3473)  
36. MIT Open Access Articles LVI-SAM: Tightly-coupled Lidar-Visual- Inertial Odometry via Smoothing and Mapping, accessed November 13, 2025, [https://dspace.mit.edu/bitstream/handle/1721.1/144047/2104.10831.pdf?sequence=2\&isAllowed=y](https://dspace.mit.edu/bitstream/handle/1721.1/144047/2104.10831.pdf?sequence=2&isAllowed=y)  
37. LIO-SAM++: A Lidar-Inertial Semantic SLAM with Association Optimization and Keyframe Selection \- MDPI, accessed November 13, 2025, [https://www.mdpi.com/1424-8220/24/23/7546](https://www.mdpi.com/1424-8220/24/23/7546)  
38. A Localization and Mapping Algorithm Based on Improved LVI-SAM for Vehicles in Field Environments \- MDPI, accessed November 13, 2025, [https://www.mdpi.com/1424-8220/23/7/3744](https://www.mdpi.com/1424-8220/23/7/3744)  
39. Semi-Elastic LiDAR-Inertial Odometry \- arXiv, accessed November 13, 2025, [https://arxiv.org/html/2307.07792v2](https://arxiv.org/html/2307.07792v2)  
40. with gps fator, the map optimization node crashed when encountering a big loop · Issue \#301 · TixiaoShan/LIO-SAM \- GitHub, accessed November 13, 2025, [https://github.com/TixiaoShan/LIO-SAM/issues/301](https://github.com/TixiaoShan/LIO-SAM/issues/301)  
41. A Fast Dynamic Point Detection Method for LiDAR-Inertial Odometry in Driving Scenarios, accessed November 13, 2025, [https://arxiv.org/html/2407.03590v1](https://arxiv.org/html/2407.03590v1)  
42. LB-LIOSAM: an improved mapping and localization method with loop detection \- Emerald, accessed November 13, 2025, [https://www.emerald.com/ir/article/52/3/381/1272508/LB-LIOSAM-an-improved-mapping-and-localization](https://www.emerald.com/ir/article/52/3/381/1272508/LB-LIOSAM-an-improved-mapping-and-localization)  
43. RF-LIO: Removal-First Tightly-coupled Lidar Inertial Odometry in High Dynamic Environments \- IEEE Xplore, accessed November 13, 2025, [https://ieeexplore.ieee.org/iel7/9635848/9635849/09636624.pdf](https://ieeexplore.ieee.org/iel7/9635848/9635849/09636624.pdf)  
44. 4.1. Inertial Estimation with Imu Preintegration — GTSAM by Example, accessed November 13, 2025, [https://gtbook.github.io/gtsam-examples/ImuFactorExample101.html](https://gtbook.github.io/gtsam-examples/ImuFactorExample101.html)  
45. gtsam::ImuFactor Class Reference, accessed November 13, 2025, [https://gtsam-jlblanco-docs.readthedocs.io/en/latest/\_static/doxygen/html/classgtsam\_1\_1ImuFactor.html](https://gtsam-jlblanco-docs.readthedocs.io/en/latest/_static/doxygen/html/classgtsam_1_1ImuFactor.html)  
46. gtsam::ImuFactor Class Reference, accessed November 13, 2025, [https://gtsam.org/doxygen/4.0.0/a03467.html](https://gtsam.org/doxygen/4.0.0/a03467.html)  
47. \[1708.03852\] VINS-Mono: A Robust and Versatile Monocular Visual ..., accessed November 13, 2025, [https://ar5iv.labs.arxiv.org/html/1708.03852](https://ar5iv.labs.arxiv.org/html/1708.03852)  
48. Marked-LIEO: Visual Marker-Aided LiDAR/IMU/Encoder Integrated Odometry \- PMC, accessed November 13, 2025, [https://pmc.ncbi.nlm.nih.gov/articles/PMC9269198/](https://pmc.ncbi.nlm.nih.gov/articles/PMC9269198/)  
49. VOX-LIO: An Effective and Robust LiDAR-Inertial Odometry System Based on Surfel Voxels, accessed November 13, 2025, [https://www.researchgate.net/publication/393224081\_VOX-LIO\_An\_Effective\_and\_Robust\_LiDAR-Inertial\_Odometry\_System\_Based\_on\_Surfel\_Voxels](https://www.researchgate.net/publication/393224081_VOX-LIO_An_Effective_and_Robust_LiDAR-Inertial_Odometry_System_Based_on_Surfel_Voxels)  
50. orthogonalization of Eigen::Affine objects needed · Issue \#243 · TixiaoShan/LIO-SAM, accessed November 13, 2025, [https://github.com/TixiaoShan/LIO-SAM/issues/243](https://github.com/TixiaoShan/LIO-SAM/issues/243)  
51. imuPreintegration.cpp issue \#436 \- TixiaoShan/LIO-SAM \- GitHub, accessed November 13, 2025, [https://github.com/TixiaoShan/LIO-SAM/issues/436](https://github.com/TixiaoShan/LIO-SAM/issues/436)  
52. IMU to LiDAR transform bug · Issue \#204 · TixiaoShan/LIO-SAM \- GitHub, accessed November 13, 2025, [https://github.com/TixiaoShan/LIO-SAM/issues/204](https://github.com/TixiaoShan/LIO-SAM/issues/204)