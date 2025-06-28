using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

public class TeleopPublisher : MonoBehaviour
{
    public string topicName = "so101_teleop/cmd_pose";
    public GameObject leftController;
    public GameObject rightController;
    public float publishMessageFrequency = 0.1f;

    private ROSConnection ros;
    private float timeElapsed;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseArrayMsg>(topicName);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;
        if (timeElapsed > publishMessageFrequency)
        {
            PublishControllerPose();
            timeElapsed = 0;
        }
    }

    private void PublishControllerPose()
    {
        PoseMsg leftPose = new PoseMsg
        {
            position = new PointMsg(
                leftController.transform.position.x,
                leftController.transform.position.y,
                leftController.transform.position.z
            ),
            orientation = new QuaternionMsg(
                leftController.transform.rotation.x,
                leftController.transform.rotation.y,
                leftController.transform.rotation.z,
                leftController.transform.rotation.w
            )
        };

        PoseMsg rightPose = new PoseMsg
        {
            position = new PointMsg(
                rightController.transform.position.x,
                rightController.transform.position.y,
                rightController.transform.position.z
            ),
            orientation = new QuaternionMsg(
                rightController.transform.rotation.x,
                rightController.transform.rotation.y,
                rightController.transform.rotation.z,
                rightController.transform.rotation.w
            )
        };

        PoseArrayMsg poseArray = new PoseArrayMsg
        {
            header = new HeaderMsg(),
            poses = new PoseMsg[] { leftPose, rightPose }
        };

        ros.Publish(topicName, poseArray);
    }
} 