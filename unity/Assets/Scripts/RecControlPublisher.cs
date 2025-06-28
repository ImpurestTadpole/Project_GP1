using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class RecControlPublisher : MonoBehaviour
{
    public string topicName = "so101_teleop/rec_control";
    public KeyCode startRecordingKey = KeyCode.R;
    public KeyCode stopRecordingKey = KeyCode.S;

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topicName);
    }

    void Update()
    {
        if (Input.GetKeyDown(startRecordingKey))
        {
            PublishCommand("start");
        }
        else if (Input.GetKeyDown(stopRecordingKey))
        {
            PublishCommand("stop");
        }
    }

    private void PublishCommand(string command)
    {
        StringMsg msg = new StringMsg(command);
        ros.Publish(topicName, msg);
        Debug.Log($"Published '{command}' to {topicName}");
    }
} 