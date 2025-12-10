# Unity for Human-Robot Interaction

## Overview

**Unity** is a professional game engine that brings photorealistic rendering, physics simulation, and AR/VR capabilities to robotics. Its strength lies in **human-robot interaction (HRI)** scenarios where visual fidelity, user interfaces, and immersive experiences are critical.

## Why Unity for Robotics?

| Feature | Benefit for Robotics |
|---------|---------------------|
| **Photorealistic Rendering** | Realistic sensor simulation (cameras, depth) |
| **HDRP (High Definition Render Pipeline)** | Real-time ray tracing, global illumination |
| **AR/VR Support** | Immersive teleoperation and training |
| **UI System** | Control panels, dashboards, HMI design |
| **Asset Store** | Pre-built environments, characters, objects |
| **Cross-Platform** | Windows, Linux, macOS, mobile, VR headsets |

## Unity vs. Gazebo

| Aspect | Gazebo | Unity |
|--------|--------|-------|
| **Purpose** | Robotics simulation | Game development + robotics |
| **Rendering** | Ogre2 (functional) | HDRP/URP (cinematic) |
| **Physics** | DART, Bullet | PhysX (NVIDIA) |
| **ROS Integration** | Native | ROS-TCP-Connector plugin |
| **Learning Curve** | Low-medium | Medium-high |
| **Use Case** | Algorithm development | HRI, visualization, training |

**When to Choose Unity**:
- Human operators need realistic feedback
- AR/VR teleoperation
- Consumer-facing robot demos
- Marketing and visualization
- Multi-sensory simulation (audio, haptics)

## Installation

### Unity Hub and Editor

```bash
# Download Unity Hub
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage

# Make executable
chmod +x UnityHub.AppImage
./UnityHub.AppImage

# Install Unity Editor (LTS version recommended)
# In Unity Hub: Installs → Add → Unity 2022.3 LTS
# Include modules: Linux Build Support, Windows Build Support
```

### Unity Robotics Hub

```bash
# Clone Unity Robotics packages
git clone https://github.com/Unity-Technologies/Unity-Robotics-Hub.git
cd Unity-Robotics-Hub
```

**Required Packages**:
- **ROS-TCP-Connector**: Unity ↔ ROS 2 communication
- **URDF Importer**: Import robot models from ROS
- **Perception**: Synthetic data generation for ML

## Setting Up ROS 2 Integration

### 1. Install ROS-TCP-Endpoint (ROS 2 Side)

```bash
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash
```

**Launch Endpoint**:
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1
```

### 2. Add ROS-TCP-Connector (Unity Side)

In Unity Editor:

1. **Window → Package Manager**
2. **+ → Add package from git URL**
3. Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
4. **Robotics → ROS Settings**:
   - Protocol: ROS 2
   - ROS IP Address: 127.0.0.1
   - ROS Port: 10000

## Importing URDF Models

### Using URDF Importer

**Install Package**:
```
Window → Package Manager → + → Add from git URL
https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

**Import Robot**:
1. **Assets → Import Robot from URDF**
2. Select your `humanoid.urdf` file
3. Configure import settings:
   - **Axis Type**: Z-Axis (ROS convention)
   - **Mesh Decomposer**: VHACD (for complex collisions)
4. Click **Import**

**Result**:
- Hierarchy: Links as GameObjects, joints as ArticulationBody
- Materials: Auto-generated from URDF colors
- Colliders: Mesh or primitive colliders

### Manual Tuning

After import, adjust:

```csharp
// Adjust joint properties
ArticulationBody joint = GetComponent<ArticulationBody>();
joint.jointFriction = 0.5f;
joint.angularDamping = 5.0f;

// Set drive parameters
ArticulationDrive drive = joint.xDrive;
drive.stiffness = 10000;
drive.damping = 500;
drive.forceLimit = 87;  // Max torque
joint.xDrive = drive;
```

## High-Fidelity Rendering

### HDRP Setup

**Create HDRP Project**:
1. Unity Hub → New Project → **High Definition RP**
2. Or convert existing: **Edit → Render Pipeline → HD Render Pipeline Wizard**

**Configure Quality**:
```
Edit → Project Settings → HDRP
- Shadow Quality: High
- Reflection Quality: High
- Screen Space Reflections: Enabled
- Ray Tracing: Enabled (if GPU supports it)
```

### Lighting for Realism

```csharp
// Directional Light (Sun)
Light sun = GameObject.Find("Directional Light").GetComponent<Light>();
sun.intensity = 100000;  // Lux
sun.colorTemperature = 6500;  // Daylight

// Volumetric Fog
Volume volume = FindObjectOfType<Volume>();
VolumetricFog fog;
if (volume.profile.TryGet(out fog))
{
    fog.meanFreePath.value = 50f;
    fog.albedo.value = Color.white;
}
```

### Materials

```csharp
// PBR Material (Physically Based Rendering)
Material metal = new Material(Shader.Find("HDRP/Lit"));
metal.SetFloat("_Metallic", 1.0f);
metal.SetFloat("_Smoothness", 0.8f);
metal.SetColor("_BaseColor", new Color(0.7f, 0.7f, 0.7f));

renderer.material = metal;
```

## Camera Simulation

### RGB Camera

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraPublisher : MonoBehaviour
{
    ROSConnection ros;
    public Camera cameraComponent;
    public string topicName = "/camera/image_raw";
    public float publishRate = 30f;

    RenderTexture renderTexture;
    Texture2D texture2D;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);

        renderTexture = new RenderTexture(640, 480, 24);
        cameraComponent.targetTexture = renderTexture;
        texture2D = new Texture2D(640, 480, TextureFormat.RGB24, false);

        InvokeRepeating(nameof(PublishImage), 0f, 1f / publishRate);
    }

    void PublishImage()
    {
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, 640, 480), 0, 0);
        texture2D.Apply();

        ImageMsg msg = new ImageMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                }
            },
            height = 480,
            width = 640,
            encoding = "rgb8",
            is_bigendian = 0,
            step = 640 * 3,
            data = texture2D.GetRawTextureData()
        };

        ros.Publish(topicName, msg);
    }
}
```

### Depth Camera

```csharp
public class DepthCameraPublisher : MonoBehaviour
{
    public Camera depthCamera;
    RenderTexture depthTexture;

    void Start()
    {
        depthCamera.depthTextureMode = DepthTextureMode.Depth;
        depthTexture = new RenderTexture(640, 480, 24, RenderTextureFormat.Depth);
        depthCamera.targetTexture = depthTexture;
    }

    void PublishDepth()
    {
        // Convert depth buffer to meters
        Shader.SetGlobalTexture("_CameraDepthTexture", depthTexture);
        // Publish to ROS as sensor_msgs/Image with encoding "32FC1"
    }
}
```

## Joint Control

### Subscribing to Joint Commands

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointController : MonoBehaviour
{
    public ArticulationBody[] joints;
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>("/joint_commands", ReceiveJointCommands);
    }

    void ReceiveJointCommands(JointStateMsg msg)
    {
        for (int i = 0; i < msg.position.Length; i++)
        {
            if (i < joints.Length)
            {
                ArticulationDrive drive = joints[i].xDrive;
                drive.target = (float)msg.position[i] * Mathf.Rad2Deg;
                joints[i].xDrive = drive;
            }
        }
    }
}
```

### Publishing Joint States

```csharp
void PublishJointStates()
{
    JointStateMsg msg = new JointStateMsg
    {
        name = new string[joints.Length],
        position = new double[joints.Length],
        velocity = new double[joints.Length],
        effort = new double[joints.Length]
    };

    for (int i = 0; i < joints.Length; i++)
    {
        msg.name[i] = joints[i].name;
        msg.position[i] = joints[i].jointPosition[0] * Mathf.Deg2Rad;
        msg.velocity[i] = joints[i].jointVelocity[0] * Mathf.Deg2Rad;
        msg.effort[i] = joints[i].jointForce[0];
    }

    ros.Publish("/joint_states", msg);
}
```

## VR Teleoperation

### Setting Up VR

**Install XR Plugin**:
```
Window → Package Manager → XR Plugin Management
- Enable OpenXR (Quest, Pico, Vive)
- Enable Oculus (Meta Quest)
```

**VR Hand Controller**:
```csharp
using UnityEngine.XR;

public class VRTeleop : MonoBehaviour
{
    public ArticulationBody leftGripper;
    InputDevice leftController;

    void Start()
    {
        leftController = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
    }

    void Update()
    {
        // Read grip button
        leftController.TryGetFeatureValue(CommonUsages.grip, out float gripValue);

        // Control gripper
        ArticulationDrive drive = leftGripper.xDrive;
        drive.target = gripValue * 45f;  // 0-45 degrees
        leftGripper.xDrive = drive;
    }
}
```

## UI and Dashboards

### Creating Control Panel

```csharp
using UnityEngine.UI;
using TMPro;

public class RobotDashboard : MonoBehaviour
{
    public TextMeshProUGUI batteryText;
    public Slider velocitySlider;
    public Button emergencyStopButton;

    void Start()
    {
        emergencyStopButton.onClick.AddListener(EmergencyStop);
        velocitySlider.onValueChanged.AddListener(OnVelocityChanged);
    }

    void UpdateBattery(float percentage)
    {
        batteryText.text = $"Battery: {percentage:F1}%";
        batteryText.color = percentage < 20 ? Color.red : Color.white;
    }

    void OnVelocityChanged(float value)
    {
        // Publish velocity command to ROS
        TwistMsg msg = new TwistMsg
        {
            linear = new Vector3Msg { x = value }
        };
        ros.Publish("/cmd_vel", msg);
    }

    void EmergencyStop()
    {
        // Send stop command
        ros.Publish("/emergency_stop", new EmptyMsg());
    }
}
```

## Performance Optimization

### 1. Level of Detail (LOD)

```csharp
LODGroup lodGroup = gameObject.AddComponent<LODGroup>();

LOD[] lods = new LOD[3];
lods[0] = new LOD(0.5f, highPolyRenderers);  // Close
lods[1] = new LOD(0.15f, mediumPolyRenderers);  // Medium
lods[2] = new LOD(0.01f, lowPolyRenderers);  // Far

lodGroup.SetLODs(lods);
lodGroup.RecalculateBounds();
```

### 2. Occlusion Culling

```
Window → Rendering → Occlusion Culling
- Bake occlusion data for static scenes
```

### 3. GPU Instancing

```csharp
Material material = renderer.sharedMaterial;
material.enableInstancing = true;  // Batch identical objects
```

## Integration with VLA Pipeline

Unity can render the **ActionService** execution:

```csharp
public class VLAVisualizer : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<ExecutionStateMsg>("/vla/execution_state", OnExecutionUpdate);
    }

    void OnExecutionUpdate(ExecutionStateMsg state)
    {
        // Visualize action progress
        if (state.current_action == "NAVIGATE")
        {
            DrawNavigationPath(state.target_pose);
        }
        else if (state.current_action == "PICK")
        {
            HighlightObject(state.target_object_id);
        }

        UpdateProgressBar(state.progress_percent);
    }
}
```

## Next Steps

- **[Chapter 4: Simulated Sensors](./04-simulated-sensors)**: Camera, lidar, IMU modeling
- **[Module 3: Isaac Sim](../03-isaac-robot-brain/01-isaac-sim-setup)**: NVIDIA's photorealistic simulation
- **[Module 4: VLA Integration](../04-vla-integration/01-voice-interface)**: Connect voice control to Unity

## Additional Resources

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-TCP-Connector Documentation](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [URDF Importer Guide](https://github.com/Unity-Technologies/URDF-Importer)
- [HDRP Manual](https://docs.unity3d.com/Packages/com.unity.render-pipelines.high-definition@latest)
- [Unity XR Documentation](https://docs.unity3d.com/Manual/XR.html)
