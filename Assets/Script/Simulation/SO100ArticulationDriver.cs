using System;
using System.Collections.Generic;
using System.Net.Mail;
using Unity.VisualScripting.Antlr3.Runtime;
using UnityEditor.Experimental.GraphView;
using UnityEditor.SearchService;
using UnityEngine;
/*
private → 他のクラスから直接アクセスさせない
stiffness → 変数名
[SerializeField] → privateでもUnityに保存させ、Inspectorに表示する
[Min(0f)] → Inspectorで入力する値の最小値を0にする

out → メソッドから呼び出し元へ、変数の値を渡すための指定
*/

/*
ros2から間接続度を受信
↓
so100bridge
↓
so100articulationdriver
↓
unityの関節が動く

unityの原罪の角度・速度
↓
so100rosbridge
↓
ros2へ送信
*/
public class SO100ArticulationDriver : MonoBehaviour
{
    [Serializable]//自作クラスの内容を、Unityが保存・表示できるようにする指定
    public class JointBinding//2つの情報をまとめたクラス
    {
        public string rosName;//ros2で使う関節名
        public ArticulationBody body;//Unity上の関節コンポーネント(Unity上のArticukationBodyへの参照を保存する)
    }
    [SerializeField]
    private JointBinding[] joints = new JointBinding[]
    {//関節情報を作り、rosNameの初期値を設定。
        new JointBinding { rosName = "shoulder_pan" },
        new JointBinding { rosName = "shoulder_lift" },
        new JointBinding { rosName = "elbow_flex" },
        new JointBinding { rosName = "wrist_flex" },
        new JointBinding { rosName = "wrist_roll" },
        new JointBinding { rosName = "gripper" }
    };
    /*
    stiffness → 目標角度からずれたときに戻そうとする強さ
    damping → 動きや振動を抑える強さ
    forcelimit → 駆動で出せる力・トルクの上限
    */
    [SerializeField, Min(0f)] private float stiffness = 100f;
    [SerializeField, Min(0f)] private float damping = 10f;
    [SerializeField, Min(0f)] private float forceLimit = 5f;

    [SerializeField, Min(0f)]
    private float targetSpeedRadPerSec = 0.5f;
    //文字列を手掛かりにして、整数を取り出す辞書(indexByName.add(joint.rosName,i)で、対応を登録する)
    //readonlyは、初期化後にこのフィールドを別の辞書へ置き換えない指定（辞書の中身へAdd()することはできる）
    private readonly Dictionary<string, int> indexByName = new Dictionary<string, int>();
    public bool IsReady {get; private set;}

    private float[] targetDegrees;

    private void Fail(string message)
    {
        Debug.LogError(message, this);
        enabled = false;
    }
    //関節の可動範囲に収める
    private static float ClampToLimits(ArticulationBody body,float degrees)
    {
        ArticulationDrive drive = body.xDrive;
        if(body.twistLock == ArticulationDofLock.LimitedMotion)
        {
            return Mathf.Clamp(degrees,drive.lowerLimit,drive.upperLimit);
        }
        return degrees;
    }
    //設定を検査して初期化する
    private void Start()
    {
        if(joints == null || joints.Length != 6)
        {
            Fail("set 6 joints");
            return;
        }

        targetDegrees = new float[joints.Length];
        var usedBodies = new HashSet<ArticulationBody>();
        for(int i= 0; i < joints.Length; i++)
        {
            JointBinding joint = joints[i];
            if(joint == null || string.IsNullOrWhiteSpace(joint.rosName) || joint.body == null)
            {
                Fail($"関節 {i} の名前またはBodyが未設定です。");
                return;
            }
            if (indexByName.ContainsKey(joint.rosName) || !usedBodies.Add(joint.body))
            {
                Fail($"関節の名前またはBodyが重複しています: {joint.rosName}");
                return;
            }
            if(joint.body.jointType != ArticulationJointType.RevoluteJoint || joint.body.dofCount != 1)
            {
                Fail($"{joint.rosName} は1自由度の回転関節にしてください。");
                return;
            }
            indexByName.Add(joint.rosName,i);
            float currentDegrees = joint.body.jointPosition[0]*Mathf.Rad2Deg;

            ArticulationDrive drive = joint.body.xDrive;
            drive.stiffness = stiffness;
            drive.damping = damping;
            drive.forceLimit = forceLimit;
            drive.targetVelocity = 0f;
            drive.target = ClampToLimits(joint.body,currentDegrees);
            joint.body.xDrive = drive;
            targetDegrees[i] = drive.target;
        }
        IsReady = true;
    }
    //受信した指令を検査して保存する（受け付けたらtrue、不正ならfalseを返す）
    public bool TrySetTargets(string[] names,double[] positionsRad)//names:受信した関節名の配列、positionsRad:各関節の目標角度
    {   
        //初期化が完了しているか
        //配列がnullでないか
        //名前と角度がそれぞれ6個あるか
        //登録されている関節名か
        //同じ関節名が重複していないか
        //角度がNaNや無限大になっていないか
        //度へ変換した値がfloatで表せるか
        if (!IsReady || 
            names == null ||
            positionsRad == null ||
            names.Length != joints.Length ||
            positionsRad.Length != joints.Length)
        {
            return false;
        }
        float[] newTargets = new float[joints.Length];
        //seen→同じ関数名が2回出てきてないかを確認する配列。最初はすべてfalse
        bool[] seen = new bool[joints.Length];

        for(int i = 0; i < names.Length; i++)
        {
            if (names[i] == null ||
                !indexByName.TryGetValue(names[i],out int index) ||//trygetvalueで、辞書から関節名を検索する
                seen[index])
            {
                return false;
            }
            double radians = positionsRad[i];
            if(double.IsNaN(radians) || double.IsInfinity(radians))//通常の角度として使えない数値を拒否する。
            {
                return false;
            }
            double degrees = radians*Mathf.Rad2Deg;//角度を計算する（ラジアン）
            if(degrees > float.MaxValue || degrees < -float.MaxValue)
            {
                return false;
            }
            newTargets[index] = ClampToLimits(joints[index].body,(float)degrees);//可動範囲に収めて、一時配列へ保存する
            seen[index] = true;
        }
        targetDegrees = newTargets;//6関節すべての検査が通ってから代入する。（途中に不正な値があっても、一部の関節だけ目標が更新されることはない。）
        return true;
    }
    //1回の物理更新で、目標角度を最大何度変更するか計算する
    private void FixedUpdate()
    {
        if(!IsReady) return;//初期化が終わっていなければ終了
        float maxStepDegrees = targetSpeedRadPerSec*Mathf.Rad2Deg*Time.fixedDeltaTime;//今回変更できる角度を計算
        //targetSpeedRadPerSec→目標角度を変える速さ
        //mathf.Rad2Deg→ラジアンから度への変換係数
        //Time.fixedDeltaTime→物理更新1回分の時間
        //全関節について設定を取り出す
        for (int i = 0; i < joints.Length; i++)
        {
            //対象の関節と、その駆動設定を取り出す
            ArticulationBody body = joints[i].body;
            ArticulationDrive drive = body.xDrive;
            //現在の駆動目標を、受信した最終目標へ少しずつ近づける。（movetowads(現在地、目標値、最大変更量)）
            drive.target = Mathf.MoveTowards(drive.target,targetDegrees[i],maxStepDegrees);
            body.xDrive = drive;//変更した駆動設定を書き戻す
        }
    }
    public void ReadState(
        out string[] names,
        out double[] positionsRad,
        out double[] velocitiesRadRerSec)
    {
        names = new string[joints.Length];
        positionsRad = new double[joints.Length];
        velocitiesRadRerSec = new double[joints.Length];
        for (int i = 0; i < joints.Length; i++)
        {
            names[i] = joints[i].rosName;
            positionsRad[i] = joints[i].body.jointPosition[0];
            velocitiesRadRerSec[i] = joints[i].body.jointVelocity[0];
        }   
    }
}