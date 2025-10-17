## 最好使用apt安装，环境自动配置
在 Ubuntu 22 上安装 CUDA 12.3 版本的步骤如下：

### 步骤 1: 添加 NVIDIA 仓库

首先，您需要确保系统的包管理器可以访问 NVIDIA 的 CUDA 仓库。

1. 打开终端，更新您的软件包列表：

   ```bash
   sudo apt update
   ```

2. 安装必需的依赖包：

   ```bash
   sudo apt install -y build-essential dkms
   ```

3. 下载并安装 CUDA 12.3 的 GPG 密钥：

   ```bash
   sudo mkdir -p /etc/apt/keyrings
   sudo curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/7fa2af80.pub | tee /etc/apt/keyrings/cuda.asc
   ```

4. 添加 CUDA 的仓库到系统源列表：

   ```bash
   sudo sh -c 'echo "deb [signed-by=/etc/apt/keyrings/cuda.asc] https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/ /" > /etc/apt/sources.list.d/cuda.list'
   ```

### 步骤 2: 安装 CUDA 12.3

1. 更新 apt 软件包列表，确保仓库信息已正确添加：

   ```bash
   sudo apt update
   ```

2. 安装 CUDA 12.3：

   ```bash
   sudo apt install -y cuda-12-3
   ```

### 步骤 3: 配置环境变量

1. 打开 `.bashrc` 文件并添加 CUDA 路径：

   ```bash
   nano ~/.bashrc
   ```

2. 在文件末尾添加以下内容：

   ```bash
   export PATH=/usr/local/cuda-12.3/bin${PATH:+:${PATH}}
   export LD_LIBRARY_PATH=/usr/local/cuda-12.3/lib64/stubs${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
   ```

3. 保存并关闭文件，应用修改：

   ```bash
   source ~/.bashrc
   ```

### 步骤 4: 验证 CUDA 安装

1. 检查 CUDA 是否已成功安装：

   ```bash
   nvcc --version
   ```

2. 如果您看到类似以下内容，说明安装成功：

   ```bash
   Cuda compilation tools, release 12.3, V12.3.0
   ```

### 步骤 5: 安装 NVIDIA 驱动（如果尚未安装）

如果您的系统尚未安装 NVIDIA 驱动，您可以使用以下命令来安装最新的驱动：

```bash
sudo apt install nvidia-driver-525
```

然后重新启动您的计算机。

这就是在 Ubuntu 22 上安装 CUDA 12.3 的完整步骤。如果有任何问题，随时告诉我！
