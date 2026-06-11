好的，我们来详细学习一下 ROS 2 Jazzy 的这篇教程：**《使用 launch\_testing 编写基础集成测试》 (Writing Basic Integration Tests with launch\_testing)**。

这篇教程的级别比前两篇高，它解决的是如何测试**多个节点交互**的场景，即**集成测试** (Integration Tests)。

-----

### 教程核心内容解析

#### 1\. 目标 (Goal)

学习如何为 ROS 2 节点（本例中为 `turtlesim`）创建和运行集成测试。

#### 2\. 背景 (Background)

  * **单元测试 (Unit Tests) vs 集成测试 (Integration Tests):**

      * **单元测试**（如上一篇 C++ GTest 教程）专注于验证一个**非常具体**的功能单元（例如一个函数、一个类）。
      * **集成测试** 专注于验证**多个代码片段之间的交互**。在 ROS 2 中，这通常意味着**启动一个或多个节点**，并测试它们之间的通信（Topics, Services 等）是否符合预期。

  * **集成测试的挑战：**

      * **并行隔离 (Isolation)：** 当你运行 `colcon test` 时，它可能会并行执行多个测试。你必须确保测试 A 的节点不会与测试 B 的节点意外通信。
          * **解决方案：** 为每个测试使用**唯一的 ROS 域 ID (ROS\_DOMAIN\_ID)**。
      * **标准化输出：** 测试需要生成标准格式的报告（如 XUnit 文件），以便 `colcon` 等工具可以解析它们。

#### 3\. 核心工具：`launch_testing`

  * `launch_testing` 是 ROS 2 中进行集成测试的主要工具。
  * 它的工作方式是**扩展 Python launch 文件**（即 `.launch.py` 文件）。
  * 它允许你在 `launch` 文件的基础上，添加两种类型的测试：
    1.  **活动测试 (Active tests)：** 在被测节点**正在运行时**执行的测试。
    2.  **关闭后测试 (Post-shutdown tests)：** 在所有被测节点**退出后**执行的测试。
  * `launch_testing` 依赖 Python 标准库中的 `unittest` 来执行实际的断言。

-----

### 4\. 集成测试的实现步骤

教程通过创建一个测试 `turtlesim_node` 的示例，讲解了如何设置一个集成测试。

#### 步骤 1：描述测试启动文件 (`.test.py`)

这是集成测试的核心。文件名通常遵循 `test/test_*.py` 模式。

  * **1.1 导入 (Imports):**

      * 除了标准的 `launch` 和 `launch_ros`，还需要导入：
        ```python
        import unittest
        import launch_testing.actions
        ```

  * **1.2 生成测试描述 (`generate_test_description`):**

      * 这个函数的作用类似于普通的 `launch` 文件，用于**描述要启动的内容**。
      * 关键区别在于，你不仅要启动被测节点，还要启动“测试本身”。

    <!-- end list -->

    ```python
    def generate_test_description():
        return (
            launch.LaunchDescription([
                # 1. 启动被测节点
                launch_ros.actions.Node(
                    package='turtlesim',
                    executable='turtlesim_node',
                    name='turtle1',
                ),
                
                # 2. 告诉测试框架“准备好开始测试”
                # 教程中用了一个 TimerAction 延迟0.5秒启动
                launch.actions.TimerAction(
                    period=0.5,
                    actions=[launch_testing.actions.ReadyToTest()],
                ),
            ]),
            {},
        )
    ```

      * `launch_testing.actions.ReadyToTest()` 是一个**信号**，它告诉 `launch_testing` 框架：被测系统已经启动，现在可以开始执行“活动测试”了。

  * **1.3 活动测试 (Active Tests):**

      * 这是在节点运行时执行的测试。
      * 你需要定义一个继承自 `unittest.TestCase` 的类。
      * `rclpy.init()` 和 `rclpy.shutdown()` 在 `setUpClass` 和 `tearDownClass` 中调用，确保所有测试共享一个 ROS 上下文。
      * `setUp` 和 `tearDown` 方法在**每个 `test_*` 方法**前后运行，用于创建/销毁测试节点，确保测试间的独立性。

    <!-- end list -->

    ```python
    class TestTurtleSim(unittest.TestCase):
        
        @classmethod
        def setUpClass(cls):
            rclpy.init()

        @classmethod
        def tearDownClass(cls):
            rclpy.shutdown()

        def setUp(self):
            # 为每个测试创建一个新的节点
            self.node = rclpy.create_node('test_turtlesim')

        def tearDown(self):
            self.node.destroy_node()

        # --- 这是一个测试用例 ---
        # proc_output 参数由 launch_testing 自动注入
        def test_publishes_pose(self, proc_output):
            """检查 pose 消息是否被发布"""
            msgs_rx = []
            sub = self.node.create_subscription(
                Pose, 'turtle1/pose', lambda msg: msgs_rx.append(msg), 100
            )
            try:
                # 等待 10 秒钟，看能否收到消息
                end_time = time.time() + 10
                while time.time() < end_time:
                    rclpy.spin_once(self.node, timeout_sec=1)
                
                # 断言：检查是否收到了足够的消息
                assert len(msgs_rx) > 100
            finally:
                self.node.destroy_subscription(sub)

        # --- 这是另一个测试用例 ---
        def test_logs_spawning(self, proc_output):
            """检查日志输出是否正常"""
            # 断言：检查 stderr 中是否包含特定字符串
            proc_output.assertWaitFor(
                'Spawning turtle [turtle1] at x=', 
                timeout=5, 
                stream='stderr'
            )
    ```

  * **1.4 关闭后测试 (Post-shutdown Tests):**

      * 这类测试在被测节点（`turtlesim_node`）关闭后运行。
      * 它们通过一个装饰器 `@launch_testing.post_shutdown_test()` 来标记。
      * 最常见的用途是：**检查节点是否正常退出**。

    <!-- end list -->

    ```python
    @launch_testing.post_shutdown_test()
    class TestTurtleSimShutdown(unittest.TestCase):
        
        # proc_info 参数由 launch_testing 注入
        def test_exit_codes(self, proc_info):
            """检查进程是否正常退出"""
            launch_testing.asserts.assertExitCodes(proc_info)
    ```

#### 步骤 2：在 `CMakeLists.txt` 中注册测试

这是实现**测试隔离**（使用独立 `ROS_DOMAIN_ID`）的关键。

  * **1. 查找依赖：**

    ```cmake
    if(BUILD_TESTING)
      find_package(ament_cmake_ros REQUIRED)
      find_package(launch_testing_ament_cmake REQUIRED)
    ```

  * **2. 定义一个辅助函数 (Helper Function)：**

      * 教程定义了一个 `add_ros_isolated_launch_test` 函数，它会自动使用 `run_test_isolated.py` 作为测试运行器 (RUNNER)。
      * `run_test_isolated.py` 会负责为这个测试分配一个唯一的 `ROS_DOMAIN_ID`。

    <!-- end list -->

    ```cmake
    function(add_ros_isolated_launch_test path)
      set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
      # add_launch_test 来自 launch_testing_ament_cmake
      add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN}) 
    endfunction()
    ```

  * **3. 添加测试：**

      * 使用这个新函数来注册你的测试 launch 文件。

    <!-- end list -->

    ```cmake
      add_ros_isolated_launch_test(test/test_integration.py) 
    endif()
    ```

#### 步骤 3：添加依赖 (`package.xml`)

你需要添加所有在测试中用到的依赖（`launch` 相关的、`rclpy` 和被测包 `turtlesim`）：

```xml
<test_depend>ament_cmake_ros</test_depend>
<test_depend>launch</test_depend>
<test_depend>launch_ros</test_depend>
<test_depend>launch_testing</test_depend>
<test_depend>launch_testing_ament_cmake</test_depend>
<test_depend>rclpy</test_depend>
<test_depend>turtlesim</test_depend>
```

#### 步骤 4：运行测试

  * 和之前一样，使用 `colcon test` 和 `colcon test-result --verbose`。

-----

### 总结

这篇教程是 ROS 2 测试的“进阶篇”，它展示了如何从**单元测试**（测试单个函数）转向**集成测试**（测试运行中的节点系统）：

1.  **核心工具：** `launch_testing`。
2.  **核心文件：** 一个 `.test.py` 结尾的 Python launch 文件。
3.  **结构：**
      * `generate_test_description()`: 启动被测节点 + 触发 `ReadyToTest()` 信号。
      * `unittest.TestCase` 类: 包含“活动测试” (`test_*` 方法)，用于在节点运行时进行订阅、发布或检查输出。
      * `@launch_testing.post_shutdown_test()` 类: 包含“关闭后测试”，用于检查节点是否正常退出。
4.  **CMake 注册：** 必须使用 `launch_testing_ament_cmake` 提供的 `add_launch_test`，并最好配合 `run_test_isolated.py` 运行器来实现 `ROS_DOMAIN_ID` 隔离。