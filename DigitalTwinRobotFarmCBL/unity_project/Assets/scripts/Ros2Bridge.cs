// ROS2Bridge.cs
// Handles ROS2 message subscription in Unity

using UnityEngine;
using ROS2;
using ROS2.Unity;
using std_msgs.msg;

public class ROS2Bridge : MonoBehaviour
{
    private ROS2UnityComponent ros2Unity;
    private ISubscription<String> instructionSub;

    public DigitalTwinManager twinManager;

    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();
    }

    void Update()
    {
        if (ros2Unity.Ok)
        {
            if (instructionSub == null)
            {
                instructionSub = ros2Unity.CreateNode("unity_node")
                    .CreateSubscription<String>(
                        "/robot_instruction",
                        msg => twinManager.ReceiveInstruction(msg.Data));
            }
        }
    }
}
