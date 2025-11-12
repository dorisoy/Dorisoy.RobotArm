#define IRB6700  // 定义编译符号，用于选择IRB6700型号机械臂（如注释掉则使用IRB4600型号）

// 系统核心命名空间
using HelixToolkit.Geometry;

// HelixToolkit 3D渲染库
using HelixToolkit.Wpf;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

// WPF相关命名空间
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;

// 3D图形相关
using System.Windows.Media.Media3D;  


/**
 * 此类负责加载机械臂所有部件的3D模型并添加到视口中
 * 同时定义机械臂各关节之间的关系，以反映真实世界中机器人的运动
 * **/
namespace Dorisoy.RobotArm;

/// <summary>
/// 关节类 - 表示机械臂的一个关节
/// </summary>
class Joint
{
    public Model3D model = null;      // 关节的3D模型
    public double angle = 0;           // 当前关节角度（度）
    public double angleMin = -180;     // 最小角度限制
    public double angleMax = 180;      // 最大角度限制
    public int rotPointX = 0;          // 旋转中心点的X坐标
    public int rotPointY = 0;          // 旋转中心点的Y坐标
    public int rotPointZ = 0;          // 旋转中心点的Z坐标
    public int rotAxisX = 0;           // 旋转轴X分量（0或1）
    public int rotAxisY = 0;           // 旋转轴Y分量（0或1）
    public int rotAxisZ = 0;           // 旋转轴Z分量（0或1）

    /// <summary>
    /// 构造函数 - 创建关节对象
    /// </summary>
    /// <param name="pModel">关节的3D模型</param>
    public Joint(Model3D pModel)
    {
        model = pModel;
    }
}

/// <summary>
/// MainWindow 主窗口类 - 机械臂3D仿真的交互逻辑
/// </summary>
public partial class MainWindow : Window
{ 
    // ========== 3D模型相关 ==========
    Model3DGroup RA = new Model3DGroup();        // 机械臂3D模型组（Robotic Arm）
    Model3D geom = null;                         // 调试用球体，用于检查关节旋转点位置
    List<Joint> joints = null;                   // 机械臂所有关节的列表
    ModelVisual3D visual;                        // 调试球体的可视化对象
    ModelVisual3D RoboticArm = new ModelVisual3D(); // 机械臂可视化对象
    
    // ========== 状态标志 ==========
    bool switchingJoint = false;                 // 是否正在切换关节（防止递归更新）
    bool isAnimating = false;                    // 是否正在执行逆运动学动画
    
    // ========== 颜色和选择相关 ==========
    Color oldColor = Colors.White;               // 之前选中模型的原始颜色
    GeometryModel3D oldSelectedModel = null;     // 之前选中的模型
    
    // ========== 路径和配置 ==========
    string basePath = "";                        // 3D模型文件的基础路径
    
    // ========== 逆运动学参数 ==========
    double LearningRate = 0.01;                  // 梯度下降的学习率
    double SamplingDistance = 0.15;              // 梯度计算的采样距离
    double DistanceThreshold = 20;               // 到达目标点的距离阈值
    Vector3D reachingPoint;                      // 目标到达点坐标
    int movements = 10;                          // 剩余移动步数
    System.Windows.Forms.Timer timer1;           // 逆运动学动画定时器
    
    // ========== 变换矩阵（用于正运动学计算） ==========
    Transform3DGroup F1;  // 关节1的累积变换
    Transform3DGroup F2;  // 关节2的累积变换
    Transform3DGroup F3;  // 关节3的累积变换
    Transform3DGroup F4;  // 关节4的累积变换
    Transform3DGroup F5;  // 关节5的累积变换
    Transform3DGroup F6;  // 关节6的累积变换
    RotateTransform3D R;  // 临时旋转变换
    TranslateTransform3D T; // 临时平移变换

#if IRB6700
    // ========== IRB6700型号机械臂的STL模型文件路径 ==========
    private const string MODEL_PATH1 = "IRB6700-MH3_245-300_IRC5_rev02_LINK01_CAD.stl";   // 链节1（基座连接件）
    private const string MODEL_PATH2 = "IRB6700-MH3_245-300_IRC5_rev00_LINK02_CAD.stl";   // 链节2（下臂）
    private const string MODEL_PATH3 = "IRB6700-MH3_245-300_IRC5_rev02_LINK03_CAD.stl";   // 链节3（上臂）
    private const string MODEL_PATH4 = "IRB6700-MH3_245-300_IRC5_rev01_LINK04_CAD.stl";   // 链节4（腕部）
    private const string MODEL_PATH5 = "IRB6700-MH3_245-300_IRC5_rev01_LINK05_CAD.stl";   // 链节5（腕部旋转）
    private const string MODEL_PATH6 = "IRB6700-MH3_245-300_IRC5_rev01_LINK06_CAD.stl";   // 链节6（法兰盘）
    private const string MODEL_PATH7 = "IRB6700-MH3_245-300_IRC5_rev02_LINK01_CABLE.stl";  // 链节1电缆
    private const string MODEL_PATH8 = "IRB6700-MH3_245-300_IRC5_rev02_LINK01m_CABLE.stl"; // 链节1电缆（镜像）
    private const string MODEL_PATH9 = "IRB6700-MH3_245-300_IRC5_rev00_LINK02_CABLE.stl";  // 链节2电缆
    private const string MODEL_PATH10 = "IRB6700-MH3_245-300_IRC5_rev00_LINK02m_CABLE.stl"; // 链节2电缆（镜像）
    private const string MODEL_PATH11 = "IRB6700-MH3_245-300_IRC5_rev00_LINK03a_CABLE.stl"; // 链节3电缆A
    private const string MODEL_PATH12 = "IRB6700-MH3_245-300_IRC5_rev00_LINK03b_CABLE.stl"; // 链节3电缆B
    private const string MODEL_PATH13 = "IRB6700-MH3_245-300_IRC5_rev02_LINK03m_CABLE.stl"; // 链节3电缆（镜像）
    private const string MODEL_PATH14 = "IRB6700-MH3_245-300_IRC5_rev01_LINK04_CABLE.stl";  // 链节4电缆
    private const string MODEL_PATH15 = "IRB6700-MH3_245-300_IRC5_rev00_ROD_CAD.stl";       // 连杆
    private const string MODEL_PATH16 = "IRB6700-MH3_245-300_IRC5_rev00_LOGO1_CAD.stl";     // Logo1
    private const string MODEL_PATH17 = "IRB6700-MH3_245-300_IRC5_rev00_LOGO2_CAD.stl";     // Logo2
    private const string MODEL_PATH18 = "IRB6700-MH3_245-300_IRC5_rev00_LOGO3_CAD.stl";     // Logo3
    private const string MODEL_PATH19 = "IRB6700-MH3_245-300_IRC5_rev01_BASE_CAD.stl";      // 基座
    private const string MODEL_PATH20 = "IRB6700-MH3_245-300_IRC5_rev00_CYLINDER_CAD.stl";   // 气缸
#else
    // ========== IRB4600型号机械臂的STL模型文件路径 ==========
    private const string MODEL_PATH1 = "IRB4600_20kg-250_LINK1_CAD_rev04.stl";    // 链节1
    private const string MODEL_PATH2 = "IRB4600_20kg-250_LINK2_CAD_rev04.stl";    // 链节2
    private const string MODEL_PATH3 = "IRB4600_20kg-250_LINK3_CAD_rev005.stl";   // 链节3
    private const string MODEL_PATH4 = "IRB4600_20kg-250_LINK4_CAD_rev04.stl";    // 链节4
    private const string MODEL_PATH5 = "IRB4600_20kg-250_LINK5_CAD_rev04.stl";    // 链节5
    private const string MODEL_PATH6 = "IRB4600_20kg-250_LINK6_CAD_rev04.stl";    // 链节6
    private const string MODEL_PATH7 = "IRB4600_20kg-250_LINK3_CAD_rev04.stl";    // 链节3（重复）
    private const string MODEL_PATH8 = "IRB4600_20kg-250_CABLES_LINK1_rev03.stl";  // 链节1电缆
    private const string MODEL_PATH9 = "IRB4600_20kg-250_CABLES_LINK2_rev03.stl";  // 链节2电缆
    private const string MODEL_PATH10 = "IRB4600_20kg-250_CABLES_LINK3_rev03.stl"; // 链节3电缆
    private const string MODEL_PATH11 = "IRB4600_20kg-250_BASE_CAD_rev04.stl";     // 基座
#endif


    /// <summary>
    /// 构造函数 - 初始化主窗口和机械臂模型
    /// </summary>
    public MainWindow()
    {
        InitializeComponent();
        
        // 获取3D模型文件的基础路径
        var currentDir = Directory.GetCurrentDirectory();
        var projectRoot = Directory.GetParent(currentDir)?.Parent?.Parent?.FullName;
        
        // 如果无法获取项目根目录（可能是发布版本），则使用应用程序基目录
        if (projectRoot == null)
        {
            projectRoot = AppDomain.CurrentDomain.BaseDirectory;
            var parentDir = Directory.GetParent(projectRoot);
            if (parentDir != null)
                projectRoot = parentDir.FullName;
        }
        
        basePath = System.IO.Path.Combine(projectRoot, "3D_Models") + System.IO.Path.DirectorySeparatorChar;
        
        // 创建模型文件名列表
        List<string> modelsNames = new List<string>();
        modelsNames.Add(MODEL_PATH1);   // 添加关节模型1-6
        modelsNames.Add(MODEL_PATH2);
        modelsNames.Add(MODEL_PATH3);
        modelsNames.Add(MODEL_PATH4);
        modelsNames.Add(MODEL_PATH5);
        modelsNames.Add(MODEL_PATH6);
        modelsNames.Add(MODEL_PATH7);   // 添加附加组件（电缆、基座等）
        modelsNames.Add(MODEL_PATH8);
        modelsNames.Add(MODEL_PATH9);
        modelsNames.Add(MODEL_PATH10);
        modelsNames.Add(MODEL_PATH11);  // IRB4600止步于此
#if IRB6700
        // IRB6700额外的组件
        modelsNames.Add(MODEL_PATH12);
        modelsNames.Add(MODEL_PATH13);
        modelsNames.Add(MODEL_PATH14);
        modelsNames.Add(MODEL_PATH15);
        modelsNames.Add(MODEL_PATH16);
        modelsNames.Add(MODEL_PATH17);
        modelsNames.Add(MODEL_PATH18);
        modelsNames.Add(MODEL_PATH19);
        modelsNames.Add(MODEL_PATH20);
#endif
        // 加载并初始化所有3D模型
        RoboticArm.Content = Initialize_Environment(modelsNames);

        /** 调试球体 - 用于检查关节旋转点的确切位置 **/
        var builder = new MeshBuilder(true, true);  // 创建网格构建器（启用法线和纹理坐标）
        var position = new System.Numerics.Vector3(0, 0, 0);  // 初始位置在原点
        builder.AddSphere(position, 50, 15, 15);  // 添加半径50的球体，15x15细分级别
        
        // 将HelixToolkit的网格转换为WPF的网格格式
        var helixMesh = builder.ToMesh();
        var wpfMesh = new System.Windows.Media.Media3D.MeshGeometry3D()
        {
            // 转换顶点位置
            Positions = new Point3DCollection(helixMesh.Positions.Select(p => new Point3D(p.X, p.Y, p.Z))),
            // 转换三角形索引
            TriangleIndices = new Int32Collection(helixMesh.TriangleIndices)
        };
        // 转换法线（如果存在）
        if (helixMesh.Normals != null)
        {
            wpfMesh.Normals = new Vector3DCollection(helixMesh.Normals.Select(n => new Vector3D(n.X, n.Y, n.Z)));
        }
        // 转换纹理坐标（如果存在）
        if (helixMesh.TextureCoordinates != null)
        {
            wpfMesh.TextureCoordinates = new PointCollection(helixMesh.TextureCoordinates.Select(t => new Point(t.X, t.Y)));
        }
        // 创建几何模型，使用棕色材质
        geom = new GeometryModel3D(wpfMesh, Materials.Brown);
        visual = new ModelVisual3D();
        visual.Content = geom;

        // 设置视口的鼠标手势
        viewPort3d.RotateGesture = new MouseGesture(MouseAction.RightClick);  // 右键旋转
        viewPort3d.PanGesture = new MouseGesture(MouseAction.LeftClick);      // 左键平移
        
        // 将调试球体和机械臂添加到3D视口
        viewPort3d.Children.Add(visual);
        viewPort3d.Children.Add(RoboticArm);
        
        // 设置相机的初始视角
        viewPort3d.Camera.LookDirection = new Vector3D(2038, -5200, -2930);  // 相机朝向
        viewPort3d.Camera.UpDirection = new Vector3D(-0.145, 0.372, 0.917); // 相机上方向
        viewPort3d.Camera.Position = new Point3D(-1571, 4801, 3774);        // 相机位置

        // 执行初始的正运动学计算，更新机械臂姿态
        double[] angles = { joints[0].angle, joints[1].angle, joints[2].angle, joints[3].angle, joints[4].angle, joints[5].angle };
        ForwardKinematics(angles);

        // 更新当前选中的关节
        changeSelectedJoint();

        // 初始化逆运动学动画定时器
        timer1 = new System.Windows.Forms.Timer();
        timer1.Interval = 5;  // 5毫秒间隔，保证平滑动画
        timer1.Tick += new System.EventHandler(timer1_Tick);  // 绑定定时器事件
    }

    /// <summary>
    /// 初始化环境 - 加载所有3D模型并配置关节参数
    /// </summary>
    /// <param name="modelsNames">模型文件名列表</param>
    /// <returns>返回装配好的机械臂3D模型组</returns>
    private Model3DGroup Initialize_Environment(List<string> modelsNames)
    {
        try
        {
            // 创建模型导入器和关节列表
            ModelImporter import = new();
            joints = [];

            // 遍历所有模型文件，加载并设置材质
            foreach(string modelName in modelsNames)
            {
                // 创建材质组（发光、漫反射、高光）
                var materialGroup = new MaterialGroup();
                Color mainColor = Colors.White;  // 主色调：白色
                EmissiveMaterial emissMat = new(new SolidColorBrush(mainColor));  // 发光材质
                DiffuseMaterial diffMat = new(new SolidColorBrush(mainColor));    // 漫反射材质
                SpecularMaterial specMat = new(new SolidColorBrush(mainColor), 250); // 高光材质，光泽度250
                materialGroup.Children.Add(emissMat);
                materialGroup.Children.Add(diffMat);
                materialGroup.Children.Add(specMat);

                // 从文件加载3D模型
                var link = import.Load(basePath + modelName);
                GeometryModel3D model = link.Children[0] as GeometryModel3D;
                model.Material = materialGroup;      // 设置正面材质
                model.BackMaterial = materialGroup;  // 设置背面材质（双面渲染）
                joints.Add(new Joint(link));         // 添加到关节列表
            }

            // 将前6个关节（主要运动部件）添加到机械臂模型组
            RA.Children.Add(joints[0].model);  // 关节1（基座旋转）
            RA.Children.Add(joints[1].model);  // 关节2（肩部）
            RA.Children.Add(joints[2].model);  // 关节3（胘部）
            RA.Children.Add(joints[3].model);  // 关节4（腕部旋转）
            RA.Children.Add(joints[4].model);  // 关节5（腕部俯仰）
            RA.Children.Add(joints[5].model);  // 关节6（法兰盘）
            RA.Children.Add(joints[6].model);  // 附加组件1
            RA.Children.Add(joints[7].model);  // 附加组件2
            RA.Children.Add(joints[8].model);  // 附加组件3
            RA.Children.Add(joints[9].model);  // 附加组件4
            RA.Children.Add(joints[10].model); // 附加组件5
#if IRB6700
            // IRB6700型号的额外组件
            RA.Children.Add(joints[11].model);
            RA.Children.Add(joints[12].model);
            RA.Children.Add(joints[13].model);
            RA.Children.Add(joints[14].model);
            RA.Children.Add(joints[15].model);
            RA.Children.Add(joints[16].model);
            RA.Children.Add(joints[17].model);
            RA.Children.Add(joints[18].model);
            RA.Children.Add(joints[19].model);
#endif

#if IRB6700
            // ========== IRB6700型号的颜色配置 ==========
            Color cableColor = Colors.DarkSlateGray;  // 电缆颜色：深石板灰
            changeModelColor(joints[6], cableColor);   // 给电缆组件上色
            changeModelColor(joints[7], cableColor);
            changeModelColor(joints[8], cableColor);
            changeModelColor(joints[9], cableColor);
            changeModelColor(joints[10], cableColor);
            changeModelColor(joints[11], cableColor);
            changeModelColor(joints[12], cableColor);
            changeModelColor(joints[13], cableColor);

            changeModelColor(joints[14], Colors.Gray);  // 连杆：灰色

            changeModelColor(joints[15], Colors.Red);   // Logo：红色
            changeModelColor(joints[16], Colors.Red);
            changeModelColor(joints[17], Colors.Red);

            changeModelColor(joints[18], Colors.Gray);  // 基座和气缸：灰色
            changeModelColor(joints[19], Colors.Gray);

            // ========== 关节1（基座旋转）的参数 ==========
            joints[0].angleMin = -180;    // 最小角度：-180°
            joints[0].angleMax = 180;     // 最大角度：180°
            joints[0].rotAxisX = 0;       // 旋转轴：Z轴（垂直旋转）
            joints[0].rotAxisY = 0;
            joints[0].rotAxisZ = 1;
            joints[0].rotPointX = 0;      // 旋转中心：原点
            joints[0].rotPointY = 0;
            joints[0].rotPointZ = 0;

            // ========== 关节2（肩部）的参数 ==========
            joints[1].angleMin = -100;    // 最小角度：-100°
            joints[1].angleMax = 60;      // 最大角度：60°
            joints[1].rotAxisX = 0;       // 旋转轴：Y轴（上下摆动）
            joints[1].rotAxisY = 1;
            joints[1].rotAxisZ = 0;
            joints[1].rotPointX = 348;    // 旋转中心位置（经过测试确定）
            joints[1].rotPointY = -243;
            joints[1].rotPointZ = 775;

            // ========== 关节3（胘部）的参数 ==========
            joints[2].angleMin = -90;     // 最小角度：-90°
            joints[2].angleMax = 90;      // 最大角度：90°
            joints[2].rotAxisX = 0;       // 旋转轴：Y轴
            joints[2].rotAxisY = 1;
            joints[2].rotAxisZ = 0;
            joints[2].rotPointX = 347;    // 旋转中心位置
            joints[2].rotPointY = -376;
            joints[2].rotPointZ = 1923;

            // ========== 关节4（腕部旋转）的参数 ==========
            joints[3].angleMin = -180;    // 最小角度：-180°
            joints[3].angleMax = 180;     // 最大角度：180°
            joints[3].rotAxisX = 1;       // 旋转轴：X轴（沿臂长旋转）
            joints[3].rotAxisY = 0;
            joints[3].rotAxisZ = 0;
            joints[3].rotPointX = 60;     // 旋转中心位置
            joints[3].rotPointY = 0;
            joints[3].rotPointZ = 2125;

            // ========== 关节5（腕部俯仰）的参数 ==========
            joints[4].angleMin = -115;    // 最小角度：-115°
            joints[4].angleMax = 115;     // 最大角度：115°
            joints[4].rotAxisX = 0;       // 旋转轴：Y轴
            joints[4].rotAxisY = 1;
            joints[4].rotAxisZ = 0;
            joints[4].rotPointX = 1815;   // 旋转中心位置
            joints[4].rotPointY = 0;
            joints[4].rotPointZ = 2125;

            // ========== 关节6（法兰盘）的参数 ==========
            joints[5].angleMin = -180;    // 最小角度：-180°
            joints[5].angleMax = 180;     // 最大角度：180°
            joints[5].rotAxisX = 1;       // 旋转轴：X轴
            joints[5].rotAxisY = 0;
            joints[5].rotAxisZ = 0;
            joints[5].rotPointX = 2008;   // 旋转中心位置
            joints[5].rotPointY = 0;
            joints[5].rotPointZ = 2125;

#else
            changeModelColor(joints[6], Colors.Red);
            changeModelColor(joints[7], Colors.Black);
            changeModelColor(joints[8], Colors.Black);
            changeModelColor(joints[9], Colors.Black);
            changeModelColor(joints[10], Colors.Gray);

            RA.Children.Add(joints[0].model);
            RA.Children.Add(joints[1].model);
            RA.Children.Add(joints[2].model);
            RA.Children.Add(joints[3].model);
            RA.Children.Add(joints[4].model);
            RA.Children.Add(joints[5].model);
            RA.Children.Add(joints[6].model);
            RA.Children.Add(joints[7].model);
            RA.Children.Add(joints[8].model);
            RA.Children.Add(joints[9].model);
            RA.Children.Add(joints[10].model);
            
            joints[0].angleMin = -180;
            joints[0].angleMax = 180;
            joints[0].rotAxisX = 0;
            joints[0].rotAxisY = 0;
            joints[0].rotAxisZ = 1;
            joints[0].rotPointX = 0;
            joints[0].rotPointY = 0;
            joints[0].rotPointZ = 0;

            joints[1].angleMin = -100;
            joints[1].angleMax = 60;
            joints[1].rotAxisX = 0;
            joints[1].rotAxisY = 1;
            joints[1].rotAxisZ = 0;
            joints[1].rotPointX = 175; 
            joints[1].rotPointY = -200;
            joints[1].rotPointZ = 500;

            joints[2].angleMin = -90;
            joints[2].angleMax = 90;
            joints[2].rotAxisX = 0;
            joints[2].rotAxisY = 1;
            joints[2].rotAxisZ = 0;
            joints[2].rotPointX = 190;
            joints[2].rotPointY = -700;
            joints[2].rotPointZ = 1595;

            joints[3].angleMin = -180;
            joints[3].angleMax = 180;
            joints[3].rotAxisX = 1;
            joints[3].rotAxisY = 0;
            joints[3].rotAxisZ = 0;
            joints[3].rotPointX = 400;
            joints[3].rotPointY = 0;
            joints[3].rotPointZ = 1765;

            joints[4].angleMin = -115;
            joints[4].angleMax = 115;
            joints[4].rotAxisX = 0;
            joints[4].rotAxisY = 1;
            joints[4].rotAxisZ = 0;
            joints[4].rotPointX = 1405;
            joints[4].rotPointY = 50;
            joints[4].rotPointZ = 1765;

            joints[5].angleMin = -180;
            joints[5].angleMax = 180;
            joints[5].rotAxisX = 1;
            joints[5].rotAxisY = 0;
            joints[5].rotAxisZ = 0;
            joints[5].rotPointX = 1405;
            joints[5].rotPointY = 0;
            joints[5].rotPointZ = 1765;
#endif
        }
        catch (Exception e)
        {
            MessageBox.Show("Exception Error:" + e.StackTrace);
        }
        return RA;
    }

    /// <summary>
    /// 数值限制函数 - 将值限制在指定范围内
    /// </summary>
    /// <typeparam name="T">可比较的数值类型</typeparam>
    /// <param name="value">待限制的值</param>
    /// <param name="min">最小值</param>
    /// <param name="max">最大值</param>
    /// <returns>限制后的值</returns>
    public static T Clamp<T>(T value, T min, T max)
        where T : System.IComparable<T>
            {
                T result = value;
                if (value.CompareTo(max) > 0)  // 如果超过最大值
                    result = max;
                if (value.CompareTo(min) < 0)  // 如果低于最小值
                    result = min;
                return result;
            }

    /// <summary>
    /// 目标点文本框变化事件 - 当用户输入目标坐标时触发
    /// </summary>
    private void ReachingPoint_TextChanged(object sender, TextChangedEventArgs e)
    {
        try
        {
            // 检查所有控件是否已初始化
            if (TbX == null || TbY == null || TbZ == null || geom == null)
                return;

            // 检查文本是否为空
            if (string.IsNullOrWhiteSpace(TbX.Text) || string.IsNullOrWhiteSpace(TbY.Text) || string.IsNullOrWhiteSpace(TbZ.Text))
                return;

            // 解析坐标并移动调试球体到目标位置
            reachingPoint = new Vector3D(Double.Parse(TbX.Text), Double.Parse(TbY.Text), Double.Parse(TbZ.Text));
            geom.Transform = new TranslateTransform3D(reachingPoint);
        }
        catch (Exception)
        {
            // 忽略解析错误
        }
    }

    /// <summary>
    /// 关节选择器值变化事件
    /// </summary>
    private void jointSelector_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
    {
        changeSelectedJoint();
    }

    /// <summary>
    /// 切换当前选中的关节 - 更新UI显示和高亮选中的模型
    /// </summary>
    private void changeSelectedJoint()
    {
        if (joints == null)
            return;

        int sel = ((int)jointSelector.Value) - 1;  // 获取选中的关节索引
        switchingJoint = true;  // 设置标志，防止递归触发事件
        unselectModel();        // 取消之前的选中
        
        if(sel < 0)  // 如果没有选中任何关节
        {
            // 禁用所有关节配置控件
            jointX.IsEnabled = false;
            jointY.IsEnabled = false;
            jointZ.IsEnabled = false;
            jointXAxis.IsEnabled = false;
            jointYAxis.IsEnabled = false;
            jointZAxis.IsEnabled = false;
        }
        else  // 选中了某个关节
        {
            // 启用关节配置控件
            if (!jointX.IsEnabled)
            {
                jointX.IsEnabled = true;
                jointY.IsEnabled = true;
                jointZ.IsEnabled = true;
                jointXAxis.IsEnabled = true;
                jointYAxis.IsEnabled = true;
                jointZAxis.IsEnabled = true;
            }
            // 显示当前关节的旋转中心坐标
            jointX.Value = joints[sel].rotPointX;
            jointY.Value = joints[sel].rotPointY;
            jointZ.Value = joints[sel].rotPointZ;
            // 显示当前关节的旋转轴
            jointXAxis.IsChecked = joints[sel].rotAxisX == 1 ? true : false;
            jointYAxis.IsChecked = joints[sel].rotAxisY == 1 ? true : false;
            jointZAxis.IsChecked = joints[sel].rotAxisZ == 1 ? true : false;
            // 高亮选中的模型
            selectModel(joints[sel].model);
            // 更新调试球体位置
            updateSpherePosition();
        }
        switchingJoint = false;  // 重置标志
    }

    /// <summary>
    /// 旋转点变化事件 - 当用户调整关节旋转中心时触发
    /// </summary>
    private void rotationPointChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
    {
        if (switchingJoint)  // 如果正在切换关节，忽略此事件
            return;

        int sel = ((int)jointSelector.Value) - 1;
        // 更新当前选中关节的旋转中心坐标
        joints[sel].rotPointX = (int)jointX.Value;
        joints[sel].rotPointY = (int)jointY.Value;
        joints[sel].rotPointZ = (int)jointZ.Value;
        updateSpherePosition();  // 更新调试球体位置
    }

    /// <summary>
    /// 更新调试球体位置 - 显示当前关节的旋转中心
    /// </summary>
    private void updateSpherePosition()
    {
        int sel = ((int)jointSelector.Value) - 1;
        if (sel < 0)
            return;

        // 创建变换组：先平移到关节旋转中心，再应用关节的当前变换
        Transform3DGroup F = new Transform3DGroup();
        F.Children.Add(new TranslateTransform3D(joints[sel].rotPointX, joints[sel].rotPointY, joints[sel].rotPointZ));
        F.Children.Add(joints[sel].model.Transform);
        geom.Transform = F;
    }

    /// <summary>
    /// 关节角度值变化事件 - 当用户调整滑块时触发
    /// </summary>
    private void joint_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
    {
        if (isAnimating)  // 如果正在执行逆运动学动画，忽略手动调整
            return;

        // 更新所有6个关节的角度
        joints[0].angle = joint1.Value;
        joints[1].angle = joint2.Value;
        joints[2].angle = joint3.Value;
        joints[3].angle = joint4.Value;
        joints[4].angle = joint5.Value;
        joints[5].angle = joint6.Value;
        execute_fk();  // 执行正运动学
    }


    /// <summary>
    /// 旋转轴复选框状态变化事件
    /// </summary>
    private void CheckBox_StateChanged(object sender, RoutedEventArgs e)
    {
        if (switchingJoint)  // 如果正在切换关节，忽略此事件
            return;

        int sel = ((int)jointSelector.Value) - 1;
        // 更新当前关节的旋转轴
        joints[sel].rotAxisX = jointXAxis.IsChecked.Value ? 1 : 0;
        joints[sel].rotAxisY = jointYAxis.IsChecked.Value ? 1 : 0;
        joints[sel].rotAxisZ = jointZAxis.IsChecked.Value ? 1 : 0;
    }


    /// <summary>
    /// 执行正运动学 - 根据关节角度计算并更新机械臂姿态
    /// 正运动学（FK）从第一个关节（基座）开始计算
    /// </summary>
    private void execute_fk()
    {
        // 收集所有6个关节的角度
        double[] angles = { joints[0].angle, joints[1].angle, joints[2].angle, joints[3].angle, joints[4].angle, joints[5].angle };
        ForwardKinematics(angles);  // 执行正运动学计算
        updateSpherePosition();      // 更新调试球体位置
    }

    /// <summary>
    /// 改变关节模型颜色
    /// </summary>
    /// <param name="pJoint">要修改颜色的关节</param>
    /// <param name="newColor">新颜色</param>
    /// <returns>返回之前的颜色</returns>
    private Color changeModelColor(Joint pJoint, Color newColor)
    {
        Model3DGroup models = ((Model3DGroup)pJoint.model);
        return changeModelColor(models.Children[0] as GeometryModel3D, newColor);
    }

    /// <summary>
    /// 改变几何模型颜色
    /// </summary>
    /// <param name="pModel">要修改颜色的几何模型</param>
    /// <param name="newColor">新颜色</param>
    /// <returns>返回之前的颜色</returns>
    private Color changeModelColor(GeometryModel3D pModel, Color newColor)
    {
        if (pModel == null)
            return oldColor;

        Color previousColor = Colors.Black;

        MaterialGroup mg = (MaterialGroup)pModel.Material;
        if (mg.Children.Count > 0)
        {
            try
            {
                // 获取并保存原始颜色
                previousColor = ((EmissiveMaterial)mg.Children[0]).Color;
                // 更新发光材质和漫反射材质的颜色
                ((EmissiveMaterial)mg.Children[0]).Color = newColor;
                ((DiffuseMaterial)mg.Children[1]).Color = newColor;
            }
            catch (Exception)
            {
                previousColor = oldColor;
            }
        }

        return previousColor;
    }


    /// <summary>
    /// 选中模型 - 高亮显示选中的模型
    /// </summary>
    /// <param name="pModel">要选中的模型</param>
    private void selectModel(Model3D pModel)
    {
        try
        {
            Model3DGroup models = ((Model3DGroup) pModel);
            oldSelectedModel = models.Children[0] as GeometryModel3D;
        }
        catch (Exception)
        {
            oldSelectedModel = (GeometryModel3D) pModel;
        }
        // 将模型颜色改为红色以高亮显示
        oldColor = changeModelColor(oldSelectedModel, ColorHelper.HexToColor("#ff3333"));
    }

    /// <summary>
    /// 取消选中模型 - 恢复原始颜色
    /// </summary>
    private void unselectModel()
    {
        changeModelColor(oldSelectedModel, oldColor);
    }

    private void ViewPort3D_OnMouseLeftButtonDown(object sender, MouseButtonEventArgs e)
    {
       Point mousePos = e.GetPosition(viewPort3d);
       PointHitTestParameters hitParams = new PointHitTestParameters(mousePos);
       VisualTreeHelper.HitTest(viewPort3d, null, ResultCallback, hitParams);
    }

    private void ViewPort3D_OnMouseLeftButtonUp(object sender, MouseButtonEventArgs e)
    {
        // Perform the hit test on the mouse's position relative to the viewport.
        HitTestResult result = VisualTreeHelper.HitTest(viewPort3d, e.GetPosition(viewPort3d));
        RayMeshGeometry3DHitTestResult mesh_result = result as RayMeshGeometry3DHitTestResult;

        if (oldSelectedModel != null)
            unselectModel();

        if (mesh_result != null)
        {
            selectModel(mesh_result.ModelHit);
        }
    }


    /// <summary>
    /// 命中测试结果回调函数 - 处理3D视口中的鼠标点击事件
    /// </summary>
    /// <param name="result">命中测试结果</param>
    /// <returns>返回命中测试行为（继续或停止）</returns>
    public HitTestResultBehavior ResultCallback(HitTestResult result)
    {
        // 检查是否命中了3D对象
        RayHitTestResult rayResult = result as RayHitTestResult;
        if (rayResult != null)
        {
            // 检查是否命中了网格几何体
            RayMeshGeometry3DHitTestResult rayMeshResult = rayResult as RayMeshGeometry3DHitTestResult;
            
            // 将调试球体移动到命中点位置
            geom.Transform = new TranslateTransform3D(new Vector3D(rayResult.PointHit.X, rayResult.PointHit.Y, rayResult.PointHit.Z));

            if (rayMeshResult != null)
            {
                // 成功命中网格几何体
                // 可以在此处添加更多的处理逻辑，例如：
                // - 显示命中点的坐标信息
                // - 高亮显示命中的模型
                // - 触发特定的交互行为
            }
        }

        // 继续命中测试，以便处理可能的其他命中对象
        return HitTestResultBehavior.Continue;
    }

    /// <summary>
    /// 启动逆运动学 - 开始或停止逆运动学动画
    /// </summary>
    public void StartInverseKinematics(object sender, RoutedEventArgs e)
    {
        if (timer1.Enabled)  // 如果已经在运行，则停止
        {
            button.Content = "前往位置";  // 按钮文字改为"前往位置"
            isAnimating = false;
            timer1.Stop();
            movements = 0;
        }
        else  // 否则启动逆运动学
        {
            geom.Transform = new TranslateTransform3D(reachingPoint);  // 将调试球移到目标位置
            movements = 5000;  // 设置最大迭代次数
            button.Content = "停止";  // 按钮文字改为"停止"
            isAnimating = true;
            timer1.Start();  // 启动定时器
        }
    }

    /// <summary>
    /// 定时器每次触发 - 执行一步逆运动学迭代
    /// </summary>
    public void timer1_Tick(object sender, EventArgs e)
    {
        // 收集当前关节角度
        double[] angles = {joints[0].angle, joints[1].angle, joints[2].angle, joints[3].angle, joints[4].angle, joints[5].angle};
        // 执行一步逆运动学，计算新的关节角度
        angles = InverseKinematics(reachingPoint, angles);
        // 更新UI滑块和关节角度
        joint1.Value = joints[0].angle = angles[0];
        joint2.Value = joints[1].angle = angles[1];
        joint3.Value = joints[2].angle = angles[2];
        joint4.Value = joints[3].angle = angles[3];
        joint5.Value = joints[4].angle = angles[4];
        joint6.Value = joints[5].angle = angles[5];

        // 减少剩余迭代次数，如果到达0则停止
        if ((--movements) <= 0)
        {
            button.Content = "Go to position";
            isAnimating = false;
            timer1.Stop();
        }
    }
    /// <summary>
    /// 逆运动学算法 - 使用梯度下降法计算达到目标点所需的关节角度
    /// </summary>
    /// <param name="target">目标位置</param>
    /// <param name="angles">当前关节角度数组</param>
    /// <returns>更新后的关节角度数组</returns>
    public double[] InverseKinematics(Vector3D target, double[] angles)
    {
        // 如果已经达到目标（距离小于阈值），停止计算
        if (DistanceFromTarget(target, angles) < DistanceThreshold)
        {
            movements = 0;
            return angles;
        }

        // 保存旧角度用于提前终止检测
        double[] oldAngles = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        angles.CopyTo(oldAngles, 0);
        
        // 对每个关节执行梯度下降
        for (int i = 0; i <= 5; i++)
        {
            // 梯度下降公式：解 -= 学习率 * 梯度
            double gradient = PartialGradient(target, angles, i);  // 计算第i个关节的梯度
            angles[i] -= LearningRate * gradient;  // 更新角度

            // 限制角度在允许范围内
            angles[i] = Clamp(angles[i], joints[i].angleMin, joints[i].angleMax);

            // 提前终止：如果已达到目标或角度不再变化
            if (DistanceFromTarget(target, angles) < DistanceThreshold || checkAngles(oldAngles, angles))
            {
                movements = 0;
                return angles;
            }
        }

        return angles;
    }

    /// <summary>
    /// 检查角度是否变化 - 用于提前终止检测
    /// </summary>
    /// <param name="oldAngles">旧角度</param>
    /// <param name="angles">新角度</param>
    /// <returns>如果角度没有变化返回true</returns>
    public bool checkAngles(double[] oldAngles, double[] angles)
    {
        for(int i = 0; i <= 5; i++)
        {
            if (oldAngles[i] != angles[i])
                return false;  // 只要有一个角度变化了，就返回false
        }

        return true;  // 所有角度都没变化
    }

    /// <summary>
    /// 偏梯度计算 - 计算第i个关节目标函数的梯度
    /// 使用数值微分法：[F(x+h) - F(x)] / h
    /// </summary>
    /// <param name="target">目标位置</param>
    /// <param name="angles">当前关节角度</param>
    /// <param name="i">要计算梯度的关节索引</param>
    /// <returns>第i个关节的梯度值</returns>
    public double PartialGradient(Vector3D target, double[] angles, int i)
    {
        // 保存原始角度，稍后会恢复
        double angle = angles[i];

        // 数值梯度： [F(x+h) - F(x)] / h
        double f_x = DistanceFromTarget(target, angles);  // F(x)

        angles[i] += SamplingDistance;  // x + h
        double f_x_plus_d = DistanceFromTarget(target, angles);  // F(x+h)

        double gradient = (f_x_plus_d - f_x) / SamplingDistance;  // 计算梯度

        // 恢复原始角度
        angles[i] = angle;

        return gradient;
    }


    /// <summary>
    /// 计算当前姿态与目标点的距离 - 使用欧几里得距离
    /// </summary>
    /// <param name="target">目标位置</param>
    /// <param name="angles">当前关节角度</param>
    /// <returns>返回距离值</returns>
    public double DistanceFromTarget(Vector3D target, double[] angles)
    {
        Vector3D point = ForwardKinematics (angles);  // 通过正运动学计算当前末端位置
        // 计算三维欧几里得距离
        return Math.Sqrt(Math.Pow((point.X - target.X), 2.0) + Math.Pow((point.Y - target.Y), 2.0) + Math.Pow((point.Z - target.Z), 2.0));
    }
    /// <summary>
    /// 正运动学 - 根据关节角度计算末端执行器的位置
    /// 使用递推变换矩阵法，从基座开始逐级计算每个关节的累积变换
    /// </summary>
    /// <param name="angles">关节角度数组（0-5对应6个关节）</param>
    /// <returns>返回末端执行器的位置坐标</returns>
    public Vector3D ForwardKinematics(double [] angles)
    {            
        // ========== 关节1（基座旋转） ==========
        // 基座只有旋转，始终在原点，所以变换组中只有旋转 R
        F1 = new Transform3DGroup();
        R = new RotateTransform3D(
            new AxisAngleRotation3D(
                new Vector3D(joints[0].rotAxisX, joints[0].rotAxisY, joints[0].rotAxisZ),  // 旋转轴
                angles[0]),  // 旋转角度
            new Point3D(joints[0].rotPointX, joints[0].rotPointY, joints[0].rotPointZ));  // 旋转中心
        F1.Children.Add(R);

        // ========== 关节2（肩部） ==========
        // 此关节附加在基座上，可能平移和旋转
        // 由于关节在STL模型中已经处于正确位置，所以初始平移为0
        // 然后在指定点执行旋转，最后应用基座的变换
        F2 = new Transform3DGroup();
        T = new TranslateTransform3D(0,0,0);  // 初始平移（保留以便未来调整）
        R = new RotateTransform3D(
            new AxisAngleRotation3D(
                new Vector3D(joints[1].rotAxisX, joints[1].rotAxisY, joints[1].rotAxisZ), 
                angles[1]), 
            new Point3D(joints[1].rotPointX, joints[1].rotPointY, joints[1].rotPointZ));
        F2.Children.Add(T);   // 平移
        F2.Children.Add(R);   // 旋转
        F2.Children.Add(F1);  // 应用上一级变换（重要！顺序不能错）

        // ========== 关节3（胘部） ==========
        // 第二个关节附加在第一个关节上，如前所述，不需要预先平移
        F3 = new Transform3DGroup();
        T = new TranslateTransform3D(0, 0, 0);
        R = new RotateTransform3D(
            new AxisAngleRotation3D(
                new Vector3D(joints[2].rotAxisX, joints[2].rotAxisY, joints[2].rotAxisZ), 
                angles[2]), 
            new Point3D(joints[2].rotPointX, joints[2].rotPointY, joints[2].rotPointZ));
        F3.Children.Add(T);
        F3.Children.Add(R);
        F3.Children.Add(F2);  // 应用上一级变换

        // ========== 关节4（腕部旋转） ==========
        F4 = new Transform3DGroup();
        T = new TranslateTransform3D(0,0,0);
        R = new RotateTransform3D(
            new AxisAngleRotation3D(
                new Vector3D(joints[3].rotAxisX, joints[3].rotAxisY, joints[3].rotAxisZ), 
                angles[3]), 
            new Point3D(joints[3].rotPointX, joints[3].rotPointY, joints[3].rotPointZ));
        F4.Children.Add(T);
        F4.Children.Add(R);
        F4.Children.Add(F3);  // 应用上一级变换

        // ========== 关节5（腕部俯仰） ==========
        F5 = new Transform3DGroup();
        T = new TranslateTransform3D(0, 0, 0);
        R = new RotateTransform3D(
            new AxisAngleRotation3D(
                new Vector3D(joints[4].rotAxisX, joints[4].rotAxisY, joints[4].rotAxisZ), 
                angles[4]), 
            new Point3D(joints[4].rotPointX, joints[4].rotPointY, joints[4].rotPointZ));
        F5.Children.Add(T);
        F5.Children.Add(R);
        F5.Children.Add(F4);  // 应用上一级变换

        // ========== 关节6（法兰盘） ==========
        // 注意：添加Children的顺序非常重要！
        // 必须先加T和R，最后加上一级变换（这是正运动学的关键）
        F6 = new Transform3DGroup();
        T = new TranslateTransform3D(0, 0, 0);
        R = new RotateTransform3D(
            new AxisAngleRotation3D(
                new Vector3D(joints[5].rotAxisX, joints[5].rotAxisY, joints[5].rotAxisZ), 
                angles[5]), 
            new Point3D(joints[5].rotPointX, joints[5].rotPointY, joints[5].rotPointZ));
        F6.Children.Add(T);
        F6.Children.Add(R);
        F6.Children.Add(F5);  // 应用上一级变换


        // ========== 应用变换到各个关节模型 ==========
        joints[0].model.Transform = F1;  // 第一个关节（基座旋转）
        joints[1].model.Transform = F2;  // 第二个关节（"二头肌"）
        joints[2].model.Transform = F3;  // 第三个关节（"膠盖"或"胘部"）
        joints[3].model.Transform = F4;  // 第四个关节（"前臂"）
        joints[4].model.Transform = F5;  // 第五个关节（工具板）
        joints[5].model.Transform = F6;  // 第六个关节（工具/法兰盘）
        
        // 显示末端执行器的实际位置（joints[5]的边界框位置）
        Tx.Content = joints[5].model.Bounds.Location.X;
        Ty.Content = joints[5].model.Bounds.Location.Y;
        Tz.Content = joints[5].model.Bounds.Location.Z;
        // 显示调试球体的位置
        Tx_Copy.Content = geom.Bounds.Location.X;
        Ty_Copy.Content = geom.Bounds.Location.Y;
        Tz_Copy.Content = geom.Bounds.Location.Z;

#if IRB6700
        // ========== IRB6700: 应用变换到附加组件 ==========
        // 这些组件（电缆、Logo等）跟随相应的关节移动
        joints[6].model.Transform = F1;   // 电缆1 -> 跟随关节1
        joints[7].model.Transform = F1;   // 电缆2 -> 跟随关节1
        joints[19].model.Transform = F1;  // 气缸 -> 跟随关节1
        joints[14].model.Transform = F1;  // 连杆 -> 跟随关节1

        joints[8].model.Transform = F2;   // 电缆3 -> 跟随关节2
        joints[9].model.Transform = F2;   // 电缆4 -> 跟随关节2

        joints[10].model.Transform = F3;  // 电缆5 -> 跟随关节3
        joints[11].model.Transform = F3;  // 电缆6 -> 跟随关节3
        joints[12].model.Transform = F3;  // 电缆7 -> 跟随关节3
        joints[16].model.Transform = F3;  // Logo2 -> 跟随关节3

        joints[13].model.Transform = F4;  // 电缆8 -> 跟随关节4
        joints[17].model.Transform = F4;  // Logo3 -> 跟随关节4
#else
        // ========== IRB4600: 应用变换到附加组件 ==========
        joints[7].model.Transform = F1;   // 电缆1 -> 跟随关节1

        joints[8].model.Transform = F2;   // 电缆2 -> 跟随关节2

        joints[6].model.Transform = F3;   // ABB标志 -> 跟随关节3
        joints[9].model.Transform = F3;   // 电缆3 -> 跟随关节3
#endif

        // 返回末端执行器的位置坐标（用于逆运动学计算）
        return new Vector3D(joints[5].model.Bounds.Location.X, joints[5].model.Bounds.Location.Y, joints[5].model.Bounds.Location.Z);
    }

}
