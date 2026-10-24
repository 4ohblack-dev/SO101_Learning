using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class SO100RosBridge : MonoBehaviour
{
    [SerializeField]
    private SO100ArticulationDriver driver;//Unityの関節を実際に動かしたり、状態を読み取ったりする担当

    [SerializeField]
    private string commandTopic = "/so100/sim/joint_commands";//ros2からの関節指令を受け取るトピック

    [SerializeField]
    private string stateTopic = "/so100/sim/joint_states";//Unityから現在の関節状態を送るトピック

    [SerializeField, Min(1f)]
    private float publishRateHz = 20f;

    private ROSConnection ros;
    private float elapsed;
    private float nextWarningTime;

    private void Start()
    {
        if (driver == null)
        {
            Debug.LogError("Driverを設定してください。", this);
            enabled = false;
            return;
        }

        ros = ROSConnection.GetOrCreateInstance();//ros接続を取得する

        ros.RegisterPublisher<JointStateMsg>(stateTopic);//現在の関節状態を送信する準備
        ros.Subscribe<JointStateMsg>(commandTopic, OnCommandReceived);//関節指令が届いたらoncommandreceivedをよぶ。
    }
    private void OnCommandReceived(JointStateMsg message)//ros2の指令を関節に渡す
    {
        if (!isActiveAndEnabled || !driver.IsReady)
            return;

        bool accepted = driver.TrySetTargets(message.name, message.position);//届いたメッセージから、関節名の配列と目標角度の配列を取り出し、driverへ渡す。

        // 不正な指令が連続しても、警告は毎秒1回までにします。
        if (!accepted && Time.unscaledTime >= nextWarningTime)
        {
            Debug.LogWarning(
                "関節指令を無視しました。" +
                "6関節の名前と角度、重複、数値を確認してください。",
                this
            );

            nextWarningTime = Time.unscaledTime + 1f;
        }
    }

    private void FixedUpdate()
    {
        if (ros == null || !driver.IsReady)
            return;

        elapsed += Time.fixedDeltaTime;

        float interval = 1f / Mathf.Max(1f, publishRateHz);

        if (elapsed < interval)
            return;

        elapsed %= interval;

        driver.ReadState(
            out string[] names,
            out double[] positions,
            out double[] velocities
        );

        JointStateMsg message = new JointStateMsg
        {
            name = names,
            position = positions,
            velocity = velocities,
            // 今回はトルクを測っていないので空配列にします。
            effort = Array.Empty<double>()
        };

        // 初期段階は実時間を使用します。
        long milliseconds = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

        message.header.stamp.sec = (int)(milliseconds / 1000);

        message.header.stamp.nanosec = (uint)((milliseconds % 1000) * 1000000);

        ros.Publish(stateTopic, message);//現在の関節状態がros2へ送られる。
    }
    //コンポーネントが破棄されるときに、関節指令の購読を解除する
    private void OnDestroy()
    {
        if (ros != null)
            ros.Unsubscribe(commandTopic);
    }
}