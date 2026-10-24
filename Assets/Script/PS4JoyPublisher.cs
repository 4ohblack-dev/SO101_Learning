using System;
using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class PS4JoyPublisher : MonoBehaviour
{
    [SerializeField] private string topicName = "/joy";

    private ROSConnection ros;
    private float nextPublishTime;

    private const float PublishInterval = 0.05f;

    void Awake()
    {
        // 確認用に、1回の入力更新で処理できる上限を16MiBにする
        InputSystem.settings.maxEventBytesPerUpdate = 16 * 1024 * 1024;
    }
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JoyMsg>(topicName);
    }

    void Update()
    {
        // 時間が来るまで送信を待つ
        if (Time.unscaledTime < nextPublishTime)
            return;

        nextPublishTime = Time.unscaledTime + PublishInterval;

        Gamepad pad = Gamepad.current;
        if (pad == null)
        {
            Debug.Log("Gamepad.current が null");
        }
        else
        {
            Debug.Log(
                $"接続: {pad.displayName}, " +
                $"左スティック: {pad.leftStick.ReadValue()}, " +
                $"×ボタン: {pad.buttonSouth.isPressed}"
            );
        }

        // 毎回、新しいメッセージを作る
        JoyMsg message = new JoyMsg();
        message.axes = new float[8];
        message.buttons = new int[10];

        if (pad != null)
        {
            Vector2 leftStick = pad.leftStick.ReadValue();
            Vector2 rightStick = pad.rightStick.ReadValue();
            Vector2 dpad = pad.dpad.ReadValue();

            message.axes[0] = leftStick.x;
            message.axes[1] = leftStick.y;
            message.axes[2] = rightStick.x;
            message.axes[3] = rightStick.y;
            message.axes[4] = pad.leftTrigger.ReadValue();
            message.axes[5] = pad.rightTrigger.ReadValue();
            message.axes[6] = dpad.x;
            message.axes[7] = dpad.y;

            message.buttons[0] = pad.buttonSouth.isPressed ? 1 : 0;
            message.buttons[1] = pad.buttonEast.isPressed ? 1 : 0;
            message.buttons[2] = pad.buttonWest.isPressed ? 1 : 0;
            message.buttons[3] = pad.buttonNorth.isPressed ? 1 : 0;
            message.buttons[4] = pad.leftShoulder.isPressed ? 1 : 0;
            message.buttons[5] = pad.rightShoulder.isPressed ? 1 : 0;
            message.buttons[6] = pad.selectButton.isPressed ? 1 : 0;
            message.buttons[7] = pad.startButton.isPressed ? 1 : 0;
            message.buttons[8] = pad.leftStickButton.isPressed ? 1 : 0;
            message.buttons[9] = pad.rightStickButton.isPressed ? 1 : 0;
        }

        // 入力を取得した時刻をROS2形式で設定する
        long milliseconds = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        message.header.stamp.sec = (int)(milliseconds / 1000);
        message.header.stamp.nanosec =
            (uint)((milliseconds % 1000) * 1000000);

        ros.Publish(topicName, message);
    }
}