

# **C++ ROS 2 新手入门精通路线图：一份精选的 GitHub 仓库学习指南**

## **引言：规划您的机器人软件学习航程**

### **现代机器人技术栈**

机器人操作系统（Robot Operating System, ROS）已成为机器人开发的行业标准。ROS 2 作为其第二代版本，针对科研、商业及工业应用中的局限性进行了重大改进，提供了更强的实时性能、安全性以及对多机器人系统的支持。它不仅仅是一个软件库，更是一个集成了工具、库和约定的庞大生态系统，旨在简化复杂机器人行为的创建过程。

### **结构化学习路径的重要性**

对于初学者而言，ROS 生态系统的广度和深度既是其优势，也带来了巨大的挑战。网络上充斥着碎片化的教程和复杂的项目，新手很容易在没有清晰方向的情况下迷失方向，最终导致学习效率低下和挫败感。本指南旨在解决这一问题，通过提供一条精心策划的、循序渐进的学习路径，引导您从最基础的概念稳步迈向复杂的自主系统实现。这里的核心理念是，真正的掌握源于对每个核心概念的独立、深入的理解，并通过实践将其联系起来。

### **如何使用本指南**

本指南被设计为一个连贯的学习课程，强烈建议您按顺序学习。每个部分都建立在前一部分知识的基础之上，从核心的软件概念无缝过渡到模拟环境中的实际机器人控制，再到高级算法的实现。成功的关键在于动手实践：克隆推荐的仓库，编译代码，运行示例，并尝试修改它们以观察结果。这将帮助您将理论知识内化为实践技能。

### **基本先决条件**

本指南假设读者具备以下基础：

* **编程语言**：对 C++ 有基本了解，包括类、函数、指针和基本数据结构。  
* **操作系统**：熟悉 Linux 命令行操作。推荐使用 Ubuntu 22.04 (Jammy Jellyfish) 或 Ubuntu 24.04 (Noble Numbat)。  
* **ROS 2 环境**：已在您的系统上安装了 ROS 2 Humble Hawksbill 或 Jazzy Jalisco 的桌面完整版（Desktop-Full）。  
* **开发工具**：安装了 Visual Studio Code 或其他您偏好的代码编辑器，并配置好 C++ 和 ROS 2 的开发环境。

为了让您对即将开始的学习旅程有一个宏观的认识，下表总结了本指南将要深入探讨的核心仓库。

**表 1：仓库快速参考**

| 仓库名称与链接 | 核心概念 | 主要语言 | 新手友好度 (1-5) | 关键学习特性 |
| :---- | :---- | :---- | :---- | :---- |
| ([https://github.com/dottantgal/ROS2\_learning](https://github.com/dottantgal/ROS2_learning)) | ROS 2 核心通信机制 | C++, Python | 5 | 每个概念一个独立包，强制练习构建系统 |
| 综合示例 (Gazebo 插件) | 机器人仿真与控制 | C++, XML | 4 | 将 ROS 话题与仿真世界中的物理运动联系起来 |
| ([https://github.com/CL2-UWaterloo/f1tenth\_ws](https://github.com/CL2-UWaterloo/f1tenth_ws)) | 闭环控制 (墙壁跟随) | C++ | 4 | 经典的 PID 控制器在机器人上的首次应用 |
| ([https://github.com/Tim-HW/ROS2-Path-Planning-Turtlebot3-Potential-Field](https://github.com/Tim-HW/ROS2-Path-Planning-Turtlebot3-Potential-Field)) | 局部路径规划 | C++ | 5 | 专为学习 C++ 和 ROS 2 设计的算法实现 |
| ([https://github.com/HiPeRT/particle\_filter](https://github.com/HiPeRT/particle_filter)) | 概率定位 (MCL) | C++ | 4 | 清晰的蒙特卡洛定位算法 C++ 实现 |
| [NaokiAkai/plain\_slam\_ros2](https://github.com/NaokiAkai/plain_slam_ros2) | SLAM 基础 | C++ | 4 | 专为教育设计的极简 SLAM 核心代码 |
| [digint/tinyfsm](https://github.com/digint/tinyfsm) | 状态机 (FSM) | C++ | 5 | 轻量级、纯 C++ 库，易于集成到 ROS 节点 |
| ([https://github.com/BehaviorTree/BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2)) | 行为树 (BT) | C++ | 4 | Nav2 和 MoveIt 2 中使用的行业标准 BT 框架 |

---

## **第一部分：基础工具箱 \- C++ 中的核心 ROS 2 概念**

在深入研究机器人算法之前，必须牢固掌握 ROS 2 的基本架构和通信模式。本部分将专注于一个组织结构极佳的仓库，它将每个核心概念分解为独立的、可编译的微型项目。这种方法不仅能教授 C++ API，还能迫使学习者反复练习 ROS 2 的整个开发周期：编写代码、配置构建系统、编译和运行。

### **1.1 您的第一个工作空间：节点、发布者和订阅者**

**学习目标**：理解 ROS 中最基本的数据流模式——创建节点，通过话题（Topics）发布数据，并由其他节点订阅这些数据。

**主要推荐仓库**：([https://github.com/dottantgal/ROS2\_learning](https://github.com/dottantgal/ROS2_learning)) 1

分析：  
该仓库对于初学者来说是无价之宝。与官方的 ros2/examples 仓库将大量示例集中在一个包中不同，ROS2\_learning 将每个核心概念都封装在一个独立的、自给自足的 ROS 2 包中。例如，CPP/01 Start with simple nodes 和 CPP/02 Publisher and subscriber 这两个目录，各自包含了独立的 CMakeLists.txt 和 package.xml 文件 1。  
这种结构具有非凡的教学价值。ROS 新手面临的主要障碍不仅是 C++ 语法，还包括整个生态系统的概念，如工作空间、功能包以及 colcon 构建系统。通过为每个新概念（如节点、发布者）都提供一个完整的、独立的包，学习者被迫在每次实践中都与构建系统打交道。他们需要理解如何在 package.xml 中声明依赖项，以及如何在 CMakeLists.txt 中找到这些依赖项并链接到可执行文件。这种重复性的、隔离的练习，是掌握从零开始构建真实机器人应用程序所需技能的最有效方法，完美契合了“循序渐进学习”的需求。

代码深度解析：  
我们将深入分析该仓库中的几个关键文件，以揭示 ROS 2 节点的基本结构。

#### **1.1.1 最简单的 ROS 2 节点**

在 CPP/01 Start with simple nodes/src/my\_first\_node.cpp 中，可以看到一个最基础的 ROS 2 节点的实现：

C++

\#**include** "rclcpp/rclcpp.hpp"

int main(int argc, char \*\*argv)  
{  
    // 1\. 初始化 ROS 2 C++ 客户端库  
    rclcpp::init(argc, argv);  
      
    // 2\. 创建一个节点实例，节点名为 "my\_first\_node"  
    auto node \= std::make\_shared\<rclcpp::Node\>("my\_first\_node");  
      
    // 3\. 打印一条日志信息，确认节点已启动  
    RCLCPP\_INFO(node-\>get\_logger(), "Hello from my first ROS 2 node\!");  
      
    // 4\. 进入事件循环，等待 ROS 2 的事件（如消息、服务请求等）  
    rclcpp::spin(node);  
      
    // 5\. 关闭 ROS 2  
    rclcpp::shutdown();  
      
    return 0;  
}

* **rclcpp::init(argc, argv)**：这是每个 ROS 2 C++ 程序的入口点，负责初始化 ROS 2 的内部资源。  
* **std::make\_shared\<rclcpp::Node\>("my\_first\_node")**：创建一个 rclcpp::Node 类的共享指针实例。ROS 2 C++ 编程广泛使用智能指针来管理内存。每个节点在 ROS 2 网络中都必须有一个唯一的名称。  
* **RCLCPP\_INFO(...)**：ROS 2 提供的日志宏，用于向控制台和日志文件输出信息。  
* **rclcpp::spin(node)**：启动节点的事件循环。这是一个阻塞调用，它会持续处理该节点的回调函数（例如，订阅消息的回调），直到节点被关闭（例如，通过 Ctrl+C）。  
* **rclcpp::shutdown()**：清理 init() 创建的资源。

#### **1.1.2 基于类的发布者与订阅者**

面向对象的编程是构建可维护的机器人软件的关键。在 CPP/02 Publisher and subscriber/src/simple\_publisher\_class\_node.cpp 中，展示了如何在一个类中实现发布者：

C++

\#**include** "rclcpp/rclcpp.hpp"  
\#**include** "std\_msgs/msg/string.hpp"  
\#**include** \<chrono\>

using namespace std::chrono\_literals;

class SimplePublisher : public rclcpp::Node  
{  
public:  
    SimplePublisher() : Node("simple\_publisher"), count\_(0)  
    {  
        // 创建一个发布者，话题名为 "chatter"，队列大小为 10  
        publisher\_ \= this\-\>create\_publisher\<std\_msgs::msg::String\>("chatter", 10);  
          
        // 创建一个定时器，每 500ms 调用一次 timer\_callback 函数  
        timer\_ \= this\-\>create\_wall\_timer(  
            500ms, std::bind(\&SimplePublisher::timer\_callback, this));  
    }

private:  
    void timer\_callback()  
    {  
        auto message \= std\_msgs::msg::String();  
        message.data \= "Hello, world\! " \+ std::to\_string(count\_++);  
        RCLCPP\_INFO(this\-\>get\_logger(), "Publishing: '%s'", message.data.c\_str());  
          
        // 发布消息  
        publisher\_-\>publish(message);  
    }

    rclcpp::TimerBase::SharedPtr timer\_;  
    rclcpp::Publisher\<std\_msgs::msg::String\>::SharedPtr publisher\_;  
    size\_t count\_;  
};

int main(int argc, char \* argv)  
{  
    rclcpp::init(argc, argv);  
    rclcpp::spin(std::make\_shared\<SimplePublisher\>());  
    rclcpp::shutdown();  
    return 0;  
}

与之对应的订阅者在 simple\_subscriber\_class\_node.cpp 中：

C++

\#**include** "rclcpp/rclcpp.hpp"  
\#**include** "std\_msgs/msg/String.hpp"

class SimpleSubscriber : public rclcpp::Node  
{  
public:  
    SimpleSubscriber() : Node("simple\_subscriber")  
    {  
        // 创建一个订阅者，订阅 "chatter" 话题  
        subscription\_ \= this\-\>create\_subscription\<std\_msgs::msg::String\>(  
            "chatter", 10, std::bind(\&SimpleSubscriber::topic\_callback, this, std::placeholders::\_1));  
    }

private:  
    void topic\_callback(const std\_msgs::msg::String::SharedPtr msg) const  
    {  
        RCLCPP\_INFO(this\-\>get\_logger(), "I heard: '%s'", msg-\>data.c\_str());  
    }  
    rclcpp::Subscription\<std\_msgs::msg::String\>::SharedPtr subscription\_;  
};

int main(int argc, char \* argv)  
{  
    rclcpp::init(argc, argv);  
    rclcpp::spin(std::make\_shared\<SimpleSubscriber\>());  
    rclcpp::shutdown();  
    return 0;  
}

* **继承 rclcpp::Node**：这是创建面向对象节点的标准做法。它允许您在类的构造函数中初始化发布者、订阅者等 ROS 2 实体。  
* **create\_publisher**：创建一个发布者对象，它被模板化为要发布的消息类型 (std\_msgs::msg::String)，并指定了话题名称和QoS（服务质量）历史深度。  
* **create\_wall\_timer**：创建一个定时器，以固定的频率触发回调函数。这是实现周期性任务（如发布传感器数据）的常用方法。  
* **create\_subscription**：创建一个订阅者，同样被模板化为消息类型。关键参数是回调函数，std::bind 用于将成员函数 topic\_callback 绑定到订阅事件上。std::placeholders::\_1 代表传入的消息。  
* **回调函数**：timer\_callback 和 topic\_callback 是事件驱动编程的核心。前者由定时器触发，后者在接收到新消息时由 ROS 2 中间件触发。

### **1.2 请求-响应模式：服务与动作**

**学习目标**：超越单向通信（话题），理解用于远程过程调用（服务）和可抢占的长时间运行任务（动作）的双向请求-响应模式。

**主要推荐仓库**：([https://github.com/dottantgal/ROS2\_learning](https://github.com/dottantgal/ROS2_learning)) 1

分析：  
我们继续使用这个优秀的仓库，重点关注 CPP/04 Service and client 和 CPP/07 Actions 目录。服务（Services）适用于快速、原子的请求-响应交互，例如“计算两个整数的和”。动作（Actions）则为需要长时间执行、提供持续反馈并可被取消的任务设计，例如“导航到目标点”。该仓库还提供了创建自定义服务（.srv）和动作（.action）定义的示例，这是定义机器人特定 API 的关键技能。  
**代码深度解析**：

#### **1.2.1 服务端与客户端**

在 CPP/04 Service and client/src/service\_node\_class.cpp 中定义了一个简单的加法服务：

C++

\#**include** "rclcpp/rclcpp.hpp"  
\#**include** "example\_interfaces/srv/add\_two\_ints.hpp"

class AddTwoIntsServer : public rclcpp::Node  
{  
public:  
    AddTwoIntsServer() : Node("add\_two\_ints\_server")  
    {  
        // 创建服务  
        server\_ \= this\-\>create\_service\<example\_interfaces::srv::AddTwoInts\>(  
            "add\_two\_ints",  
            std::bind(\&AddTwoIntsServer::handle\_service, this,  
                      std::placeholders::\_1, std::placeholders::\_2));  
        RCLCPP\_INFO(this\-\>get\_logger(), "Service 'add\_two\_ints' is ready.");  
    }

private:  
    void handle\_service(  
        const std::shared\_ptr\<example\_interfaces::srv::AddTwoInts::Request\> request,  
        std::shared\_ptr\<example\_interfaces::srv::AddTwoInts::Response\> response)  
    {  
        RCLCPP\_INFO(this\-\>get\_logger(), "Incoming request: a=%ld, b=%ld", request-\>a, request-\>b);  
        response-\>sum \= request-\>a \+ request-\>b;  
    }  
    rclcpp::Service\<example\_interfaces::srv::AddTwoInts\>::SharedPtr server\_;  
};

客户端代码见 client\_node\_class.cpp：

C++

\#**include** "rclcpp/rclcpp.hpp"  
\#**include** "example\_interfaces/srv/add\_two\_ints.hpp"

class AddTwoIntsClient : public rclcpp::Node  
{  
public:  
    AddTwoIntsClient() : Node("add\_two\_ints\_client")  
    {  
        // 创建客户端  
        client\_ \= this\-\>create\_client\<example\_interfaces::srv::AddTwoInts\>("add\_two\_ints");  
        // 等待服务上线  
        while (\!client\_-\>wait\_for\_service(std::chrono::seconds(1))) {  
            RCLCPP\_WARN(this\-\>get\_logger(), "Service not available, waiting again...");  
        }  
    }

    // 发送请求的方法  
    void send\_request(int64\_t a, int64\_t b)  
    {  
        auto request \= std::make\_shared\<example\_interfaces::srv::AddTwoInts::Request\>();  
        request-\>a \= a;  
        request-\>b \= b;

        // 异步发送请求  
        auto future\_result \= client\_-\>async\_send\_request(request);  
          
        // 等待结果  
        if (rclcpp::spin\_until\_future\_complete(this\-\>get\_node\_base\_interface(), future\_result) \==  
            rclcpp::FutureReturnCode::SUCCESS)  
        {  
            RCLCPP\_INFO(this\-\>get\_logger(), "Sum: %ld", future\_result.get()-\>sum);  
        } else {  
            RCLCPP\_ERROR(this\-\>get\_logger(), "Failed to call service add\_two\_ints");  
        }  
    }

private:  
    rclcpp::Client\<example\_interfaces::srv::AddTwoInts\>::SharedPtr client\_;  
};

* **create\_service**：在服务端节点中创建服务。回调函数 handle\_service 接受请求和响应对象作为参数。  
* **create\_client**：在客户端节点中创建客户端。  
* **async\_send\_request**：客户端使用此函数异步发送请求。它返回一个 future 对象，可以通过 spin\_until\_future\_complete 等待其完成。

#### **1.2.2 动作服务端与客户端**

动作的实现更为复杂，因为它涉及目标、反馈和结果三个阶段。CPP/07 Actions/src/class\_action\_server.cpp 展示了动作服务端的核心逻辑：

C++

// 伪代码，展示核心结构  
class FibonacciActionServer : public rclcpp::Node  
{  
public:  
    //... 构造函数...  
    FibonacciActionServer() : Node("fibonacci\_action\_server")  
    {  
        action\_server\_ \= rclcpp\_action::create\_server\<Fibonacci\>(  
            this, "fibonacci",  
            std::bind(\&FibonacciActionServer::handle\_goal, this, \_1, \_2),  
            std::bind(\&FibonacciActionServer::handle\_cancel, this, \_1),  
            std::bind(\&FibonacciActionServer::handle\_accepted, this, \_1));  
    }

private:  
    // 决定是否接受新目标  
    rclcpp\_action::GoalResponse handle\_goal(...) { /\*... \*/ return rclcpp\_action::GoalResponse::ACCEPT\_AND\_EXECUTE; }  
      
    // 处理取消请求  
    rclcpp\_action::CancelResponse handle\_cancel(...) { /\*... \*/ return rclcpp\_action::CancelResponse::ACCEPT; }  
      
    // 当目标被接受后，在新线程中执行任务  
    void handle\_accepted(const std::shared\_ptr\<GoalHandleFibonacci\> goal\_handle)  
    {  
        std::thread{std::bind(\&FibonacciActionServer::execute, this, \_1), goal\_handle}.detach();  
    }

    // 执行长时间任务的函数  
    void execute(const std::shared\_ptr\<GoalHandleFibonacci\> goal\_handle)  
    {  
        auto feedback \= std::make\_shared\<Fibonacci::Feedback\>();  
        //... 执行任务，周期性发布反馈...  
        for (int i \= 1; (i \< goal.order) && rclcpp::ok(); \++i)  
        {  
            // 检查是否被取消  
            if (goal\_handle-\>is\_canceling()) {  
                //... 设置结果为已取消并返回...  
                return;  
            }  
            //... 计算斐波那契数列...  
            feedback-\>partial\_sequence.push\_back(sequence\[i\]);  
            goal\_handle-\>publish\_feedback(feedback); // 发布反馈  
            rate.sleep();  
        }  
          
        // 任务完成，发送结果  
        auto result \= std::make\_shared\<Fibonacci::Result\>();  
        result-\>sequence \= sequence;  
        goal\_handle-\>succeed(result); // 成功  
    }  
};

动作客户端则需要处理反馈和最终结果的回调。这种模式非常适合机器人导航、机械臂运动等任务。

### **1.3 配置与坐标系统：参数与 TF2**

**学习目标**：学习如何在不重新编译代码的情况下配置节点（参数），以及如何管理机器人不同部件之间的空间关系（TF2）。

**主要推荐仓库**：([https://github.com/dottantgal/ROS2\_learning](https://github.com/dottantgal/ROS2_learning)) 1

分析：  
最后，我们考察 CPP/05 Parameters 和 CPP/10 TF2 dynamic broadcaster 目录。参数（Parameters）是 ROS 2 的一个关键特性，它允许在运行时动态调整节点的行为，例如调整控制器的增益或设置传感器的处理阈值。TF2 是 ROS 中用于处理坐标变换的库，它是任何移动机器人应用的基石，负责回答“激光雷达坐标系中的一个点在地图坐标系中的位置是什么？”这类问题。  
**代码深度解析**：

#### **1.3.1 在 C++ 节点中使用参数**

CPP/05 Parameters/src/set\_parameters.cpp 展示了如何声明和使用参数：

C++

\#**include** "rclcpp/rclcpp.hpp"

class SimpleParameter : public rclcpp::Node  
{  
public:  
    SimpleParameter() : Node("simple\_parameter")  
    {  
        // 声明一个名为 "my\_parameter" 的参数，默认值为 "world"  
        this\-\>declare\_parameter\<std::string\>("my\_parameter", "world");  
          
        timer\_ \= this\-\>create\_wall\_timer(  
            std::chrono::seconds(1),  
            std::bind(\&SimpleParameter::timer\_callback, this));  
    }

private:  
    void timer\_callback()  
    {  
        // 获取参数值  
        std::string my\_param \= this\-\>get\_parameter("my\_parameter").as\_string();  
          
        RCLCPP\_INFO(this\-\>get\_logger(), "Hello %s", my\_param.c\_str());  
    }  
    rclcpp::TimerBase::SharedPtr timer\_;  
};

您可以在启动节点时通过命令行或启动文件（launch file）来设置这个参数的值，从而改变节点的行为。

#### **1.3.2 广播 TF2 变换**

TF2 的核心是广播（broadcasting）和监听（listening）坐标系之间的变换关系。CPP/10 TF2 dynamic broadcaster/src/tf2\_publisher\_node.cpp 展示了如何广播一个动态变换：

C++

\#**include** "rclcpp/rclcpp.hpp"  
\#**include** "geometry\_msgs/msg/transform\_stamped.hpp"  
\#**include** "tf2/LinearMath/Quaternion.h"  
\#**include** "tf2\_ros/transform\_broadcaster.h"

class DynamicTFBroadcaster : public rclcpp::Node  
{  
public:  
    DynamicTFBroadcaster() : Node("dynamic\_tf\_broadcaster")  
    {  
        // 初始化变换广播器  
        broadcaster\_ \= std::make\_shared\<tf2\_ros::TransformBroadcaster\>(this);  
        timer\_ \= this\-\>create\_wall\_timer(  
            std::chrono::milliseconds(100),  
            std::bind(\&DynamicTFBroadcaster::broadcast\_timer\_callback, this));  
    }

private:  
    void broadcast\_timer\_callback()  
    {  
        geometry\_msgs::msg::TransformStamped t;

        // 填充变换信息  
        t.header.stamp \= this\-\>get\_clock()-\>now();  
        t.header.frame\_id \= "world"; // 父坐标系  
        t.child\_frame\_id \= "robot\_frame"; // 子坐标系

        // 假设机器人正在绕 Z 轴旋转  
        double time\_sec \= this\-\>get\_clock()-\>now().seconds();  
        t.transform.translation.x \= 1.0 \* sin(time\_sec);  
        t.transform.translation.y \= 1.0 \* cos(time\_sec);  
        t.transform.translation.z \= 0.0;

        tf2::Quaternion q;  
        q.setRPY(0, 0, time\_sec); // 绕 Z 轴旋转  
        t.transform.rotation.x \= q.x();  
        t.transform.rotation.y \= q.y();  
        t.transform.rotation.z \= q.z();  
        t.transform.rotation.w \= q.w();

        // 广播变换  
        broadcaster\_-\>sendTransform(t);  
    }  
      
    std::shared\_ptr\<tf2\_ros::TransformBroadcaster\> broadcaster\_;  
    rclcpp::TimerBase::SharedPtr timer\_;  
};

这个节点会周期性地发布一个从 world 坐标系到 robot\_frame 坐标系的变换，表示机器人正在绕着原点做圆周运动。其他节点（如 RViz 或定位模块）可以监听这些变换，从而实时了解机器人的位姿。

---

## **第二部分：从代码到运动 \- 仿真与基础控制**

在掌握了 ROS 2 的软件基础之后，下一步是让代码产生实际的物理效果。本部分将指导您进入 Gazebo 仿真环境，这是一个强大的物理模拟器，能让您在编写一行硬件驱动代码之前就测试和验证机器人算法。我们将从生成一个简单的机器人模型开始，最终实现一个经典的闭环控制任务。

### **2.1 Gazebo 简介：生成并控制一个简单的机器人**

**学习目标**：理解如何启动 Gazebo 仿真世界，加载一个机器人模型（URDF），并通过 ROS 2 话题来控制它的运动。

**主要推荐示例**：综合官方文档和插件示例

分析：  
ROS 2 与 Gazebo 的集成是通过一系列插件完成的。对于移动机器人，最重要的插件之一是差分驱动插件（gazebo\_ros\_diff\_drive）。这个插件扮演着“虚拟电机控制器”的角色。它在 Gazebo 内部运行，订阅一个 geometry\_msgs/msg/Twist 类型的 ROS 2 话题（通常是 /cmd\_vel），并将接收到的线速度和角速度指令转换为施加在仿真机器人轮子上的力和力矩，从而驱动机器人运动 2。  
这个过程完美地展示了 ROS 的模块化思想：您的控制节点（无论是键盘遥控还是自主算法）只需要关心发布标准的 Twist 消息，而无需了解底层硬件或仿真的具体实现。

**代码深度解析**：

#### **2.1.1 在 URDF 中添加差分驱动插件**

要在 Gazebo 中控制您的机器人，您需要在其 URDF (Unified Robot Description Format) 或 SDF (Simulation Description Format) 文件中添加并配置 gazebo\_ros\_diff\_drive 插件。以下是一个典型的配置片段 2：

XML

\<gazebo\>  
  \<plugin name\="gazebo\_ros\_diff\_drive" filename\="libgazebo\_ros\_diff\_drive.so"\>  
      
    \<update\_rate\>30\</update\_rate\>

    \<left\_joint\>left\_wheel\_joint\</left\_joint\>  
    \<right\_joint\>right\_wheel\_joint\</right\_joint\>

    \<wheel\_separation\>0.4\</wheel\_separation\>  
    \<wheel\_diameter\>0.2\</wheel\_diameter\>

    \<odometry\_frame\>odom\</odometry\_frame\>  
    \<robot\_base\_frame\>base\_link\</robot\_base\_frame\>  
    \<publish\_odom\>true\</publish\_odom\>  
    \<publish\_odom\_tf\>true\</publish\_odom\_tf\>  
    \<publish\_wheel\_tf\>true\</publish\_wheel\_tf\>

    \<command\_topic\>cmd\_vel\</command\_topic\>

  \</plugin\>  
\</gazebo\>

* **left\_joint / right\_joint**：指定 URDF 中定义的左右轮关节的名称。  
* **wheel\_separation / wheel\_diameter**：定义机器人的物理尺寸，用于运动学计算。  
* **odometry\_frame / robot\_base\_frame**：定义里程计 TF 变换的坐标系名称。  
* **command\_topic**：指定插件监听速度指令的话题名称。

#### **2.1.2 发布 /cmd\_vel 指令**

一旦机器人在 Gazebo 中加载了此插件，您就可以通过编写一个简单的发布者节点来控制它。这个节点与我们在第一部分中创建的 SimplePublisher 非常相似，只是发布的消息类型和内容不同。

C++

\#**include** "rclcpp/rclcpp.hpp"  
\#**include** "geometry\_msgs/msg/twist.hpp"

class RobotController : public rclcpp::Node  
{  
public:  
    RobotController() : Node("robot\_controller")  
    {  
        publisher\_ \= this\-\>create\_publisher\<geometry\_msgs::msg::Twist\>("cmd\_vel", 10);  
          
        // 创建一个一次性定时器，2秒后调用 move\_robot  
        timer\_ \= this\-\>create\_wall\_timer(  
            std::chrono::seconds(2), std::bind(\&RobotController::move\_robot, this));  
    }

private:  
    void move\_robot()  
    {  
        auto twist\_msg \= geometry\_msgs::msg::Twist();  
          
        // 设置线速度为 0.5 m/s (沿 X 轴)  
        twist\_msg.linear.x \= 0.5;  
        // 设置角速度为 0.2 rad/s (绕 Z 轴)  
        twist\_msg.angular.z \= 0.2;  
          
        RCLCPP\_INFO(this\-\>get\_logger(), "Publishing Twist command: linear.x=%.2f, angular.z=%.2f",  
                    twist\_msg.linear.x, twist\_msg.angular.z);  
          
        publisher\_-\>publish(twist\_msg);  
    }

    rclcpp::Publisher\<geometry\_msgs::msg::Twist\>::SharedPtr publisher\_;  
    rclcpp::TimerBase::SharedPtr timer\_;  
};

int main(int argc, char \*\*argv)  
{  
    rclcpp::init(argc, argv);  
    rclcpp::spin(std::make\_shared\<RobotController\>());  
    rclcpp::shutdown();  
    return 0;  
}

当您运行这个节点时，它会发布一条指令，使 Gazebo 中的机器人以 0.5 m/s 的速度前进，并以 0.2 rad/s 的角速度逆时针旋转。这是一个**开环控制**的例子：我们发送了一个指令，但没有根据机器人的状态或环境反馈来调整它。

### **2.2 实现基本的反应式行为：墙壁跟随算法**

**学习目标**：创建一个闭环控制器，利用传感器数据（激光雷达）生成控制指令（速度），以实现一个简单的自主行为。

**主要推荐仓库**：([https://github.com/CL2-UWaterloo/f1tenth\_ws](https://github.com/CL2-UWaterloo/f1tenth_ws)) 3

分析：  
从开环控制到闭环控制，是机器人学入门过程中最关键的认知飞跃。开环控制是盲目的，而闭环控制则引入了“感知-思考-行动”（Sense-Think-Act）的循环，这是所有自主行为的基础。墙壁跟随算法是实践这一理念的完美入门项目。它的任务很简单：利用激光雷达（LiDAR）测量与墙壁的距离，并调整机器人的转向，使其与墙壁保持一个固定的距离。  
CL2-UWaterloo/f1tenth\_ws 仓库是一个为 F1TENTH 自动驾驶赛车平台设计的综合项目，但其 src/wall\_follow 包是一个小巧、独立的 PID 控制器实现，非常适合教学 3。F1TENTH 课程也将墙壁跟随作为一个基础实验，证明了其在教学中的重要性。

这个项目的核心是建立一个反馈循环：

1. **感知 (Sense)**：订阅 /scan 话题，获取 sensor\_msgs/msg/LaserScan 消息。  
2. **思考 (Think)**：处理激光数据，计算当前与墙壁的误差，并通过 PID 控制算法计算出必要的转向角度。  
3. **行动 (Act)**：将计算出的转向角度和设定的前进速度打包成 geometry\_msgs/msg/Twist 消息，发布到 /cmd\_vel 话题。

通过研究这个简单的、一维输入（墙壁距离）到一维输出（转向角）的控制问题，学习者可以牢固地建立起对反馈控制的直观理解，为后续更复杂的算法打下坚实的基础。

代码深度解析：  
我们将分析 wall\_follow 包中 C++ 源代码的核心逻辑（基于其功能描述）。

C++

// 伪代码，展示 wall\_follow 节点的核心结构和逻辑  
\#**include** "rclcpp/rclcpp.hpp"  
\#**include** "sensor\_msgs/msg/laser\_scan.hpp"  
\#**include** "geometry\_msgs/msg/twist.hpp"

class WallFollower : public rclcpp::Node  
{  
public:  
    WallFollower() : Node("wall\_follower")  
    {  
        // PID 控制器参数  
        this\-\>declare\_parameter\<double\>("kp", 1.0);  
        this\-\>declare\_parameter\<double\>("ki", 0.0);  
        this\-\>declare\_parameter\<double\>("kd", 0.1);  
        this\-\>declare\_parameter\<double\>("desired\_distance", 1.0);  
        this\-\>declare\_parameter\<double\>("forward\_speed", 0.5);

        kp\_ \= this\-\>get\_parameter("kp").as\_double();  
        ki\_ \= this\-\>get\_parameter("ki").as\_double();  
        kd\_ \= this\-\>get\_parameter("kd").as\_double();  
        desired\_distance\_ \= this\-\>get\_parameter("desired\_distance").as\_double();  
        forward\_speed\_ \= this\-\>get\_parameter("forward\_speed").as\_double();

        // 订阅激光雷达数据  
        scan\_sub\_ \= this\-\>create\_subscription\<sensor\_msgs::msg::LaserScan\>(  
            "scan", 10, std::bind(\&WallFollower::scan\_callback, this, std::placeholders::\_1));  
          
        // 发布速度指令  
        vel\_pub\_ \= this\-\>create\_publisher\<geometry\_msgs::msg::Twist\>("cmd\_vel", 10);  
          
        prev\_error\_ \= 0.0;  
        integral\_ \= 0.0;  
    }

private:  
    void scan\_callback(const sensor\_msgs::msg::LaserScan::SharedPtr msg)  
    {  
        // 1\. 从 LaserScan 数据中计算当前与墙壁的距离  
        // (简化逻辑：假设墙在右侧，取右侧扫描点的平均值)  
        // 实际实现会更复杂，需要找到合适的扫描角度  
        double current\_distance \= get\_distance\_to\_wall(msg);

        // 2\. 计算误差  
        double error \= desired\_distance\_ \- current\_distance;

        // 3\. 实现 PID 控制律  
        integral\_ \+= error;  
        double derivative \= error \- prev\_error\_;  
          
        double steering\_angle \= kp\_ \* error \+ ki\_ \* integral\_ \+ kd\_ \* derivative;

        prev\_error\_ \= error;

        // 4\. 创建并发布 Twist 消息  
        auto twist\_msg \= geometry\_msgs::msg::Twist();  
        twist\_msg.linear.x \= forward\_speed\_;  
        twist\_msg.angular.z \= steering\_angle;  
          
        vel\_pub\_-\>publish(twist\_msg);  
    }

    // 辅助函数，用于从激光数据中提取墙壁距离  
    double get\_distance\_to\_wall(const sensor\_msgs::msg::LaserScan::SharedPtr scan)  
    {  
        //... 此处为处理激光数据的具体逻辑...  
        // 例如，找到与机器人侧面成特定角度的激光束，并返回其距离  
        // 简化示例：返回 90 度方向的激光束距离  
        size\_t index \= scan-\>ranges.size() / 4; // 假设 90 度在 1/4 处  
        return scan-\>ranges\[index\];  
    }

    rclcpp::Subscription\<sensor\_msgs::msg::LaserScan\>::SharedPtr scan\_sub\_;  
    rclcpp::Publisher\<geometry\_msgs::msg::Twist\>::SharedPtr vel\_pub\_;  
      
    double kp\_, ki\_, kd\_;  
    double desired\_distance\_;  
    double forward\_speed\_;  
    double prev\_error\_, integral\_;  
};

* **PID 控制律**：代码的核心是 steering\_angle \= kp\_ \* error \+ ki\_ \* integral\_ \+ kd\_ \* derivative; 这一行。  
  * **比例项 (P)**：kp\_ \* error，误差越大，转向调整越大。这是最主要的控制部分。  
  * **积分项 (I)**：ki\_ \* integral\_，累积过去的误差，用于消除稳态误差（例如，由于系统偏差，机器人总是离墙壁稍微远一点）。  
  * **微分项 (D)**：kd\_ \* derivative，基于误差的变化率进行调整，用于抑制过冲和振荡，使系统响应更平滑。

通过这个项目，您将第一次真正地赋予机器人“智能”，使其能够根据环境变化做出反应。

---

## **第三部分：构建自主能力 \- C++ 中的关键机器人算法**

在掌握了基础控制之后，我们将深入研究构成现代机器人自主能力的三个核心支柱：路径规划、定位和建图。本部分精选的仓库遵循“恰到好处”的原则：它们足够复杂以至于有实际意义，但又足够小巧和独立，使得初学者能够完全理解其 C++ 源代码。这种学习方式远比直接深入研究像 Nav2 或 slam\_toolbox 这样庞大、生产级的框架要有效得多。

学习大型框架对于初学者来说可能是一次令人沮丧的经历，因为核心算法逻辑往往被层层插件、接口和生命周期管理代码所包裹。相比之下，下面这些仓库是“算法的实现”，而非“功能完备的产品”。这使得学习者可以在合理的时间内通读并理解一个完整算法的 C++ 代码，从而建立起深入的理解和自信，为将来使用或贡献于更复杂的框架做好准备。

### **3.1 路径规划：使用势场法从 A 点导航到 B 点**

**学习目标**：实现一个局部路径规划器，能够引导机器人在避开障碍物的同时朝向目标点移动。

**主要推荐仓库**：([https://github.com/Tim-HW/ROS2-Path-Planning-Turtlebot3-Potential-Field](https://github.com/Tim-HW/ROS2-Path-Planning-Turtlebot3-Potential-Field)) 4

分析：  
这个仓库是初学者的理想选择，因为其作者明确表示，创建这个项目就是为了学习 C++ 和 ROS 2。它实现了一种经典且直观的算法——势场法（Potential Field Method）。该方法将机器人置于一个虚拟的力场中：目标点产生一个吸引力，拉着机器人前进；而环境中的障碍物则产生排斥力，推开机器人。机器人在任何时刻的运动方向，就是这些吸引力和排斥力的合力方向。  
代码深度解析：  
我们将分析该仓库 src 目录下的 C++ 源代码，揭示其核心逻辑。

C++

// 伪代码，展示势场法节点的核心逻辑  
class PotentialFieldPlanner : public rclcpp::Node  
{  
public:  
    PotentialFieldPlanner() : Node("potential\_field\_planner")  
    {  
        //... 参数声明：吸引力增益, 排斥力增益, 障碍物影响范围...  
        this\-\>declare\_parameter\<double\>("attractive\_gain", 1.0);  
        this\-\>declare\_parameter\<double\>("repulsive\_gain", 2.0);  
        this\-\>declare\_parameter\<double\>("obstacle\_radius", 0.5);

        //... 订阅激光雷达和目标点话题...  
        scan\_sub\_ \= this\-\>create\_subscription\<sensor\_msgs::msg::LaserScan\>(  
            "scan", 10, std::bind(\&PotentialFieldPlanner::scan\_callback, this, \_1));  
        goal\_sub\_ \= this\-\>create\_subscription\<geometry\_msgs::msg::PoseStamped\>(  
            "goal\_pose", 10, std::bind(\&PotentialFieldPlanner::goal\_callback, this, \_1));  
          
        //... 发布速度指令...  
        vel\_pub\_ \= this\-\>create\_publisher\<geometry\_msgs::msg::Twist\>("cmd\_vel", 10);  
    }

private:  
    void scan\_callback(const sensor\_msgs::msg::LaserScan::SharedPtr msg)  
    {  
        // 1\. 计算吸引力 (如果目标点已知)  
        Vector2D attractive\_force \= {0.0, 0.0};  
        if (goal\_received\_) {  
            // 假设机器人当前位姿为 (0,0)  
            Vector2D goal\_vector \= {goal\_pos\_.x, goal\_pos\_.y};  
            attractive\_force \= attractive\_gain\_ \* goal\_vector;  
        }

        // 2\. 计算排斥力  
        Vector2D repulsive\_force \= {0.0, 0.0};  
        for (size\_t i \= 0; i \< msg-\>ranges.size(); \++i) {  
            double dist \= msg-\>ranges\[i\];  
            if (dist \< obstacle\_radius\_) {  
                double angle \= msg-\>angle\_min \+ i \* msg-\>angle\_increment;  
                // 计算障碍物在机器人坐标系下的位置  
                Vector2D obstacle\_vector \= {dist \* cos(angle), dist \* sin(angle)};  
                // 排斥力与障碍物方向相反，大小与距离成反比  
                repulsive\_force \-= repulsive\_gain\_ \* (1.0/dist \- 1.0/obstacle\_radius\_) \* (1.0/pow(dist, 2)) \* obstacle\_vector.normalize();  
            }  
        }

        // 3\. 计算合力  
        Vector2D total\_force \= attractive\_force \+ repulsive\_force;

        // 4\. 将合力转换为 Twist 消息  
        auto twist\_msg \= geometry\_msgs::msg::Twist();  
        // 线速度与合力在机器人前进方向(X轴)的分量成正比  
        twist\_msg.linear.x \= std::min(max\_speed\_, total\_force.x);  
        // 角速度与合力在机器人侧向(Y轴)的分量或合力向量的角度成正比  
        twist\_msg.angular.z \= atan2(total\_force.y, total\_force.x);  
          
        vel\_pub\_-\>publish(twist\_msg);  
    }

    //... 其他成员变量和 goal\_callback...  
};

* **吸引力计算**：代码计算从机器人当前位置指向目标点的向量，并乘以一个增益系数。这是一个简单的线性吸引力模型。  
* **排斥力计算**：遍历激光雷达的每一个扫描点。如果某个点的距离小于设定的影响范围，就计算一个沿该激光束方向向外的排斥力。这个力的大小通常与距离障碍物的远近成反比，越近力越大。  
* **合力与控制**：将吸引力和所有排斥力向量相加，得到最终的合力向量。这个向量的方向就是机器人期望的运动方向。最后，将这个二维力向量转换为一维的线速度和角速度指令。

### **3.2 定位：使用粒子滤波器回答“我在哪里？”**

**学习目标**：理解并实现一种概率定位算法，用于在已知地图中估计机器人的位姿。

**主要推荐仓库**：([https://github.com/HiPeRT/particle\_filter](https://github.com/HiPeRT/particle_filter))

分析：  
定位是移动机器人导航的先决条件。粒子滤波器（Particle Filter），也称为蒙特卡洛定位（Monte Carlo Localization, MCL），是一种非常流行且强大的概率定位方法。其基本思想是用大量带权重的随机样本（即“粒子”）来表示机器人可能位姿的概率分布。每个粒子都代表一个关于“机器人可能在这里”的假设。  
HiPeRT/particle\_filter 仓库提供了一个用于 2D 激光雷达定位的 C++ 参考实现。它清晰地展示了粒子滤波器的完整工作流程，包括运动模型更新、传感器模型更新、粒子加权、位姿估计和重采样，是学习该算法的绝佳资源。

代码深度解析：  
粒子滤波器的核心在于一个循环更新过程：

C++

// 伪代码，展示粒子滤波器节点的核心更新循环  
class ParticleFilterNode : public rclcpp::Node  
{  
    //... 构造函数，订阅 odom 和 scan...  
private:  
    void update\_filter(const nav\_msgs::msg::Odometry::SharedPtr odom\_msg,  
                       const sensor\_msgs::msg::LaserScan::SharedPtr scan\_msg)  
    {  
        // 1\. 运动模型更新 (Prediction Step)  
        motion\_model\_update(odom\_msg);

        // 2\. 传感器模型更新 (Correction Step)  
        sensor\_model\_update(scan\_msg);

        // 3\. 重采样 (Resampling)  
        resample\_particles();

        // 4\. 估计并发布机器人位姿  
        publish\_pose\_estimate();  
    }

    // 对每个粒子应用运动模型  
    void motion\_model\_update(const nav\_msgs::msg::Odometry::SharedPtr odom\_msg)  
    {  
        // 计算自上次更新以来的里程计位移  
        Transform delta\_odom \= calculate\_odometry\_change(odom\_msg);

        for (auto& particle : particles\_) {  
            // 将里程计位移应用到每个粒子上  
            particle.pose \= particle.pose \* delta\_odom;  
            // 添加随机噪声，模拟运动的不确定性  
            add\_motion\_noise(particle);  
        }  
    }

    // 根据传感器读数计算每个粒子的权重  
    void sensor\_model\_update(const sensor\_msgs::msg::LaserScan::SharedPtr scan\_msg)  
    {  
        double total\_weight \= 0.0;  
        for (auto& particle : particles\_) {  
            // 计算该粒子位姿下的期望激光读数与实际读数的匹配程度  
            // 这通常通过在地图上进行射线投射（ray casting）来完成  
            double probability \= calculate\_scan\_likelihood(particle.pose, scan\_msg, map\_);  
            particle.weight \= probability;  
            total\_weight \+= particle.weight;  
        }

        // 归一化权重  
        for (auto& particle : particles\_) {  
            particle.weight /= total\_weight;  
        }  
    }

    // 根据权重重新生成粒子群  
    void resample\_particles()  
    {  
        // 实现低方差重采样或其他算法  
        // 权重越高的粒子，越有可能被复制到新的粒子集中  
        // 权重低的粒子则被淘汰  
        //...  
    }

    //... 其他成员变量和函数...  
    std::vector\<Particle\> particles\_;  
};

* **运动模型 (Prediction)**：根据里程计数据预测机器人的新位置。对每个粒子应用相同的位移，并加上一些随机噪声，以表示里程计的误差和运动的不确定性。  
* **传感器模型 (Correction)**：这是算法的核心。对于每个粒子（即每个位姿假设），计算在该位姿下，机器人“应该”看到的激光扫描是什么样的（通过与地图比较）。然后，将这个“期望扫描”与机器人实际接收到的激光扫描进行比较。两者越匹配，该粒子的权重就越高。  
* **重采样 (Resampling)**：根据计算出的权重，重新生成粒子群。权重高的粒子会被多次复制，而权重低的粒子则可能被丢弃。这一步使得粒子群向高概率区域集中，最终收敛到机器人的真实位姿。

### **3.3 建图：SLAM 入门**

**学习目标**：通过研究一个极简的实现，掌握同步定位与建图（Simultaneous Localization and Mapping, SLAM）的基本概念。

**主要推荐仓库**：NaokiAkai/plain\_slam\_ros2

分析：  
SLAM 是机器人学的“圣杯”问题之一：让一个机器人在未知环境中，在没有先验地图的情况下，一边构建环境地图，一边利用这张不完整的地图来估计自身的实时位置。  
plain\_slam\_ros2 是一个专为“研究和教育”设计的项目，其核心 C++ 代码不足 1800 行，这对于学习者来说是一个巨大的优势。它依赖轻量（如 Eigen, Sophus, nanoflann），并且核心算法逻辑与 ROS 2 解耦，使得研究算法本身变得非常方便 。该项目实现了 SLAM 的关键组成部分，包括激光雷达-惯性里程计（LIO）、基于 GICP 的回环检测和位姿图优化。

代码深度解析：  
由于 SLAM 算法的复杂性，我们在此不展示完整的代码，而是概述其数据流和模块组织。  
该项目的 C++ 源码被组织成一个接口层和多个核心模块：

1. **ROS 接口层 (lio\_3d\_node.cpp, slam\_3d\_node.cpp)**：  
   * 这些节点负责与 ROS 2 世界交互。  
   * 它们订阅 sensor\_msgs/msg/PointCloud2 (来自激光雷达) 和 sensor\_msgs/msg/Imu (来自惯性测量单元) 话题。  
   * 它们将 ROS 消息转换为内部数据结构，并传递给核心 SLAM 模块。  
   * 它们从核心模块获取计算出的位姿和地图，并将其发布为 ROS 话题（如 /tf, /map）。  
2. **核心 SLAM 模块 (Plain SLAM Components)**：  
   * **前端 (LIO)**：处理连续的激光雷达扫描和 IMU 数据，以高频率估计机器人的相对运动。这部分负责生成里程计。它同时提供了松耦合和紧耦合两种 LIO 方法的实现。  
   * **后端 (Pose Graph Optimization)**：  
     * **回环检测 (Loop Detection)**：当机器人回到一个之前经过的地方时，该模块会识别出来。它通过比较当前激光扫描与历史关键帧的扫描来实现。  
     * **位姿图优化 (Pose Graph Optimization)**：一旦检测到回环，后端会创建一个约束（“我确定当前位置就是之前经过的那个位置”），并将其添加到由所有历史位姿组成的位姿图中。然后，它会运行一个优化算法，调整所有历史位姿，以最小化整个轨迹的累积误差，从而得到一个全局一致的地图和轨迹。

通过研究这个仓库的配置文件（.yaml 文件），您可以了解如何调整算法参数，例如点云处理的细节、回环检测的阈值等，这对于理解 SLAM 系统的内部工作至关重要。

---

## **第四部分：高级系统架构 \- 组织复杂的行为**

当机器人需要执行的任务变得复杂时，简单的 if/else 逻辑或单个回调函数将不足以管理其行为。本部分将介绍两种强大的设计模式——状态机和行为树，它们是构建健壮、可扩展和可维护的机器人应用程序的基石。

这里的学习路径经过精心设计。我们首先介绍一个通用的、轻量级的 C++ 状态机库，让您在不被 ROS 框架细节干扰的情况下，掌握状态机的核心思想。亲身体验过状态机的结构和局限性后，再引入作为现代 ROS 2 机器人（如 Nav2）行为调度标准 Behaviors Trees，您将能更深刻地体会到其模块化和反应式设计的优越性。这种递进式的学习方法，不仅教您“如何做”，更让您理解“为什么这样做”。

### **4.1 管理机器人逻辑：C++ 状态机概述**

**学习目标**：理解如何使用有限状态机（Finite State Machine, FSM）来管理机器人的操作状态（例如，空闲、导航中、执行任务、充电）。

**主要推荐仓库**：

分析：  
状态机是一种数学模型，它将复杂的行为分解为一系列离散的、有限的“状态”，以及在这些状态之间转换的“转移”。例如，一个清洁机器人可能具有 正在充电、前往清洁区、正在清洁 和 返回充电桩 等状态。  
虽然 ROS 生态中有一些专用的 FSM 库，如 packml\_ros2 和 SMACC2，但它们通常与特定的工业标准或复杂的框架绑定，对于初学者来说学习曲线陡峭。相比之下，TinyFSM 是一个简单、纯头文件、无依赖的 C++11 状态机库。它的优势在于其通用性和轻量级。通过将这个通用库集成到一个 ROS 2 节点中，学习者可以清晰地将“状态机逻辑”与“ROS 通信逻辑”分离开来，这是一种更纯粹、更高效的学习体验。

代码深度解析：  
我们将展示如何在一个 ROS 2 节点中使用 TinyFSM 来管理一个简单的机器人行为。

#### **4.1.1 使用 TinyFSM 定义状态和事件**

TinyFSM 使用 C++ 模板元编程来定义状态机，这使得定义非常紧凑。

#### **4.1.2 将 FSM 集成到 ROS 2 节点中**

现在，我们可以在一个 ROS 2 节点中实例化并驱动这个状态机。

在这个例子中，ROS 节点的订阅者回调函数负责将传入的 ROS 消息转换为 TinyFSM 的事件，并使用 dispatch 函数来驱动状态转移。反过来，在每个状态的 entry() 或 exit() 动作中，您可以调用 ROS 2 的功能，比如发布一个话题或调用一个服务，从而将状态机的内部逻辑与机器人的外部行为联系起来。

### **4.2 可组合的反应式逻辑：行为树简介**

**学习目标**：学习行为树（Behavior Trees, BTs）的基础知识，这是一种比 FSM 更强大、更灵活的工具，用于创建复杂的、模块化的、反应式的机器人行为。

**主要推荐仓库**：() (核心库) 和() (ROS 2 集成)

分析：  
行为树已经成为 ROS 2 中任务级控制的事实标准，被 Nav2 和 MoveIt 2 等核心项目广泛采用。与状态机不同，行为树本质上是分层的、可组合的。您可以创建代表简单行为的“叶节点”（例如，“检查电池电量”、“移动到A点”），然后使用“控制节点”（如 Sequence, Fallback, Parallel）将它们组合成复杂的任务树。这种模块化的特性使得添加新功能或修改现有逻辑变得异常简单，通常只需在树上添加或替换一个分支，而无需像 FSM 那样重构大量的状态转移。  
BehaviorTree.CPP 是一个高性能的 C++17 库，是该领域的行业标杆。而 BehaviorTree.ROS2 仓库则提供了与 ROS 2 无缝集成的封装器，让您可以轻松地在行为树中调用 ROS 2 的动作、服务和话题。

**代码深度解析**：

#### **4.2.1 行为树的基本概念**

* **Action 节点**：执行一个动作并返回 SUCCESS, FAILURE, 或 RUNNING。例如，MoveBaseAction。  
* **Condition 节点**：检查一个条件并立即返回 SUCCESS 或 FAILURE。例如，IsBatteryLow。  
* **Sequence 节点 (-\>)**：按顺序执行其子节点。只要有子节点返回 FAILURE 或 RUNNING，它就停止并返回该状态。只有所有子节点都返回 SUCCESS，它才返回 SUCCESS。  
* **Fallback 节点 (?)**：按顺序执行其子节点。只要有子节点返回 SUCCESS 或 RUNNING，它就停止并返回该状态。只有所有子节点都返回 FAILURE，它才返回 FAILURE。

行为树通常用 XML 文件来定义，这使得逻辑与代码分离，可以在运行时加载和修改。

这个树的逻辑是：尝试执行“巡逻序列”。如果电池电量 OK (IsBatteryOK 返回 SUCCESS)，则依次移动到路点 A 和 B。如果 IsBatteryOK 返回 FAILURE，Sequence 节点将失败，Fallback 节点会转而尝试执行 GoToChargingStation。

#### **4.2.2 在 C++ 中创建并集成 ROS 2 行为节点**

BehaviorTree.ROS2 库极大地简化了 ROS 2 功能的集成。以下是如何创建一个封装了 ROS 2 动作客户端的自定义行为树动作节点的示例，该示例改编自官方文档：

#### **4.2.3 注册并运行行为树**

在您的主 C++ 节点中，您需要注册这个自定义节点，然后加载 XML 文件并“tick”树来执行它。

这个例子展示了行为树框架的强大之处：大部分与 ROS 2 动作客户端交互的复杂异步逻辑（发送目标、处理反馈、等待结果、处理取消）都被 RosActionNode 基类封装好了。您只需要实现几个关键的回调函数，就可以将任何 ROS 2 动作无缝集成到您的机器人行为逻辑中。

---

## **结论：您的前进之路**

### **学习之旅总结**

本指南为您铺设了一条从零开始掌握 ROS 2 和 C++ 的结构化学习路径。我们从 ROS 2 的基础通信机制（节点、话题、服务、动作）出发，通过 dottantgal/ROS2\_learning 仓库，强调了在实践中掌握构建系统的重要性。接着，我们进入 Gazebo 仿真世界，实现了从简单的开环控制到第一个基于传感器的闭环自主行为——墙壁跟随，完成了从纯软件到机器人运动的关键一步。

随后，我们深入探讨了构成机器人自主能力的几大核心算法。通过学习 Tim-HW/ROS2-Path-Planning-Turtlebot3-Potential-Field、HiPeRT/particle\_filter 和 NaokiAkai/plain\_slam\_ros2 等小巧而专注的仓库，您得以在不被庞大框架淹没的情况下，理解路径规划、定位和 SLAM 的核心 C++ 实现。最后，我们介绍了用于构建复杂、健壮行为的两种高级架构模式：状态机和行为树，并展示了如何将通用 C++ 库 TinyFSM 和行业标准的 BehaviorTree.CPP 集成到 ROS 2 应用中。

### **新的征程**

完成本指南中的所有步骤后，您将具备坚实的 ROS 2 C++ 开发基础，并有能力去探索更广阔的机器人学领域。接下来的学习方向可以包括：

* **机器人操纵**：深入研究 MoveIt 2 框架，学习运动规划、逆运动学和机械臂控制。  
* **高级感知**：使用点云库（PCL）处理 3D 传感器数据，实现物体识别和场景理解。  
* **导航与探索**：深入研究 Nav2 框架的内部工作原理，学习如何配置和扩展其规划器、控制器和行为树。  
* **开源贡献**：尝试为您在本指南中使用过的某个仓库修复一个 bug 或添加一个小功能，这是提升技能并融入社区的最佳方式。

为了给您的学习提供一个清晰、可执行的计划，下表将整个指南的内容浓缩成一个循序渐进的学习路径。

**表 2：循序渐进的学习路径**

机器人学是一个充满挑战但回报丰厚的领域。坚持不懈地动手实践，保持好奇心，您将能够利用本指南所授的知识，构建出真正智能和有用的机器人系统。祝您学习顺利！

#### **Works cited**

1. dottantgal/ROS2\_learning: A useful collection of CPP and ... \- GitHub, accessed October 19, 2025, [https://github.com/dottantgal/ROS2\_learning](https://github.com/dottantgal/ROS2_learning)  
2. Robot Operating System: Controlling a Robot with the ROS Gazebo ..., accessed October 19, 2025, [https://admantium.com/blog/ros09\_controlling\_a\_robot\_in\_gazebo/](https://admantium.com/blog/ros09_controlling_a_robot_in_gazebo/)  
3. CL2-UWaterloo/f1tenth\_ws: A repository for autonomous ... \- GitHub, accessed October 19, 2025, [https://github.com/CL2-UWaterloo/f1tenth\_ws](https://github.com/CL2-UWaterloo/f1tenth_ws)  
4. Tim-HW/ROS2-Path-Planning-Turtlebot3-Potential-Field ... \- GitHub, accessed October 19, 2025, [https://github.com/Tim-HW/ROS2-Path-Planning-Turtlebot3-Potential-Field](https://github.com/Tim-HW/ROS2-Path-Planning-Turtlebot3-Potential-Field)  
5. NaokiAkai/plain\_slam\_ros2: Lightweight LiDAR-Inertial ... \- GitHub, accessed October 19, 2025, [https://github.com/NaokiAkai/plain\_slam\_ros2](https://github.com/NaokiAkai/plain_slam_ros2)