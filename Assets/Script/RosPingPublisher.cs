using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class RosPingPublisher : MonoBehaviour
{
    ROSConnection ros;
    const string TopicName = "unity_ping";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(TopicName);
    }

    void Update()
    {
        if (Time.frameCount % 60 == 0)
            ros.Publish(TopicName, new StringMsg("Hello from Unity"));
    }
}