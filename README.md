# RobotArmHelix - 机械臂3D仿真系统

基于 C# WPF 和 HelixToolkit 的工业机器人3D仿真系统，支持正运动学和逆运动学计算。

![机械臂仿真界面](https://raw.githubusercontent.com/Gabryxx7/RobotArmHelix/master/Images/robotArmHelix.png)

## 📋 项目简介

这是一个功能完整的机器人机械臂3D仿真应用程序，使用 C# 和 WPF 开发，通过 [HelixToolkit](https://github.com/helix-toolkit/helix-toolkit) 实现高性能3D渲染。项目已成功迁移至 **.NET 8**，支持最新的开发工具和运行时环境。

### ✨ 核心功能

- **3D可视化渲染** - 使用 HelixToolkit 渲染高质量的机械臂3D模型
- **正运动学(FK)** - 根据关节角度计算末端执行器位置
- **逆运动学(IK)** - 基于梯度下降算法，自动计算到达目标点所需的关节角度
- **实时交互控制** - 通过滑块实时调整每个关节的角度
- **可视化调试** - 调试球体显示关节旋转中心，便于调试
- **双机型支持** - 支持 ABB IRB 4600 和 IRB 6700 两种机械臂型号

### 🎯 技术特点

- **算法实现**：逆运动学基于梯度下降优化算法（参考 [Alan Zucconi 的教程](http://www.alanzucconi.com/2017/04/10/robotic-arms/)）
- **模型来源**：使用 ABB 官方 CAD 模型，通过 FreeCAD 转换为 STEP 格式，再用 MeshLab 简化为约 10,000 面的 STL 网格，确保渲染性能
- **架构设计**：清晰的代码结构，详细的中文注释，易于理解和扩展

## 🛠️ 技术栈

- **.NET 8** - 最新的 .NET 平台
- **WPF** - Windows Presentation Foundation UI框架
- **C#** - 编程语言
- **HelixToolkit 3.1.1** - 3D图形渲染库
- **Visual Studio 2022** (推荐) 或 JetBrains Rider

## 📦 环境要求

- **操作系统**：Windows 10/11 (x64)
- **.NET SDK**：.NET 8.0 或更高版本
- **IDE**：Visual Studio 2022、Visual Studio Code 或 JetBrains Rider
- **显卡**：支持 DirectX 11 的显卡（用于3D渲染）

## 🚀 快速开始

### 1. 安装 .NET 8 SDK

如果尚未安装，请从 [Microsoft 官网](https://dotnet.microsoft.com/download/dotnet/8.0) 下载并安装 .NET 8 SDK。

验证安装：
```bash
dotnet --version
```

### 2. 克隆或下载项目

```bash
git clone https://github.com/Gabryxx7/RobotArmHelix.git
cd RobotArmHelix
```

或直接下载 ZIP 并解压。

### 3. 还原依赖项

```bash
cd RobotArmHelix
dotnet restore
```

### 4. 编译项目

**Debug 模式：**
```bash
dotnet build
```

**Release 模式：**
```bash
dotnet build --configuration Release
```

### 5. 运行程序

```bash
dotnet run
```

或者直接运行生成的可执行文件：
```bash
.\bin\Release\net8.0-windows\RobotArmHelix.exe
```

## 📖 使用说明

### 界面布局

- **左侧控制面板**
  - **目标位置输入** (X, Y, Z)：输入末端执行器的目标坐标
  - **关节角度滑块** (J1-J6)：手动调整每个关节的角度 (-180° ~ 180°)
  - **关节选择器**：选择要调试的关节，显示其旋转中心和旋转轴
  - **旋转点坐标** (X, Y, Z)：当前选中关节的旋转中心坐标
  - **旋转轴选择** (X, Y, Z)：当前选中关节的旋转轴方向

- **中央3D视口**
  - **鼠标右键** + 拖动：旋转视角
  - **鼠标左键** + 拖动：平移视角
  - **鼠标滚轮**：缩放视图

### 操作流程

#### 正运动学（手动控制）

1. 使用左侧的 **J1-J6 滑块** 调整各关节角度
2. 3D视口中的机械臂会实时更新姿态
3. 界面显示末端执行器的实际位置坐标

#### 逆运动学（自动到达目标点）

1. 在顶部的 **X, Y, Z 文本框** 中输入目标位置（例如：X=1500, Y=0, Z=1750）
2. 点击 **"Go to position"** 按钮
3. 系统自动使用梯度下降算法计算关节角度
4. 机械臂会平滑移动到目标位置
5. 点击 **"STOP"** 按钮可随时停止

#### 关节调试

1. 使用 **Joint 滑块** 选择要调试的关节（1-6）
2. 被选中的关节会以红色高亮显示
3. 调试球体（棕色）会移动到该关节的旋转中心
4. 可调整旋转点坐标和旋转轴方向，实时查看效果

## 🔧 配置选项

### 切换机械臂型号

在 `MainWindow.xaml.cs` 文件的第一行可以切换机型：

```csharp
#define IRB6700  // 使用 IRB6700 型号
// 或注释掉以使用 IRB4600 型号
```

### 逆运动学参数调整

在 `MainWindow.xaml.cs` 中可调整以下参数：

```csharp
double LearningRate = 0.01;        // 学习率（较小值更精确但更慢）
double SamplingDistance = 0.15;    // 梯度采样距离
double DistanceThreshold = 20;     // 到达目标的距离阈值（单位：毫米）
```

## 📁 项目结构

```
RobotArmHelix/
├── RobotArmHelix/
│   ├── 3D_Models/              # STL 模型文件
│   │   ├── IRB4600_*.stl      # IRB4600 型号模型
│   │   └── IRB6700_*.stl      # IRB6700 型号模型
│   ├── Properties/
│   │   └── AssemblyInfo.cs    # 程序集信息
│   ├── App.xaml               # 应用程序定义
│   ├── App.xaml.cs
│   ├── MainWindow.xaml        # 主窗口UI
│   ├── MainWindow.xaml.cs     # 主窗口逻辑（核心代码）
│   └── RobotArmHelix.csproj   # 项目文件
└── README.md
```

## 🧮 算法原理

### 正运动学 (Forward Kinematics)

使用递推变换矩阵法，从基座开始逐级计算每个关节的累积变换：

```
F1 = R1
F2 = T2 * R2 * F1
F3 = T3 * R3 * F2
...
F6 = T6 * R6 * F5
```

最终得到末端执行器的位置和姿态。

### 逆运动学 (Inverse Kinematics)

基于梯度下降优化算法：

1. **目标函数**：距离函数 `f(θ) = ||FK(θ) - target||²`
2. **梯度计算**：使用数值微分法计算每个关节角度的偏导数
3. **迭代更新**：`θᵢ = θᵢ - α * ∇f(θᵢ)`
4. **约束限制**：每次更新后将角度限制在关节的有效范围内
5. **提前终止**：当距离小于阈值或角度不再变化时停止

## 🎨 自定义扩展

### 添加新的机械臂模型

1. 准备 STL 格式的3D模型文件
2. 放置到 `3D_Models/` 目录
3. 在 `MainWindow.xaml.cs` 中添加模型路径常量
4. 配置关节参数（角度限制、旋转轴、旋转中心）
5. 在 `Initialize_Environment` 方法中加载模型

### 修改材质和颜色

在 `Initialize_Environment` 方法中使用 `changeModelColor` 函数：

```csharp
changeModelColor(joints[0], Colors.Blue);  // 将关节1改为蓝色
```

## 🐛 故障排除

### 问题：找不到3D模型文件

**解决方案**：确保 `3D_Models` 文件夹位于项目根目录，且包含所有 `.stl` 文件。

### 问题：逆运动学无法收敛

**解决方案**：
- 检查目标点是否在机械臂的工作空间内
- 适当降低 `LearningRate`
- 增加 `DistanceThreshold`

### 问题：3D渲染性能低

**解决方案**：
- 使用 MeshLab 进一步简化 STL 模型面数
- 更新显卡驱动
- 关闭其他占用GPU的程序

## 📚 参考资源

- [HelixToolkit 官方文档](https://github.com/helix-toolkit/helix-toolkit)
- [逆运动学算法教程 - Alan Zucconi](http://www.alanzucconi.com/2017/04/10/robotic-arms/)
- [ABB IRB 4600 CAD 模型](http://new.abb.com/products/robotics/industrial-robots/irb-4600/irb-4600-cad)
- [RAV2 项目](https://github.com/karthikram827/RAV2) - 原始灵感来源

## 📝 版本历史

- **v2.0** (2024) - 迁移至 .NET 8，添加详细中文注释，优化路径处理
- **v1.0** (2017) - 初始版本，支持 .NET Framework 4.8.1

## 📄 许可证

本项目基于原作者 Gabriele Marini (Gabryxx7) 的开源项目。

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 👨‍💻 原作者

- **Gabriele Marini (Gabryxx7)** - 原始项目开发者
- GitHub: [@Gabryxx7](https://github.com/Gabryxx7)

---

**注意**：本项目仅供学习和研究使用。3D模型版权归 ABB 公司所有。
