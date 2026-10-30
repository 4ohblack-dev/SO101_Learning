#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "SMS_STS.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>

using namespace std::chrono_literals;//時間に関して、20ms、300msのように書けるようにする

//サーボとの通信クラス（SMS_STSクラスを継承している）
class ServoBus : public SMS_STS
{
public:
    ~ServoBus()//デストラクタ（ServoBusオブジェクトが破棄されるとき、自動的に呼ばれる）
    {//fd → 開いたシリアルポートをosが識別するための番号（0以上なら有効な番号として扱う）
        if(fd >= 0){
            ::close(fd);//Linuxのclose()を呼びだしてポートを閉じる（::は、グローバル名前空間の関数を指定する）
            fd = -1;//有効なポートを持っていない状態にする
        }
    }
    ServoBus() = default;//引数なしのコンストラクタを、コンパイラが作る標準の処理に任せる。
    ServoBus(const ServoBus &) = delete;//コピーのコンストラクタを禁止（複数のオブジェクトが同じポート番号を持ち、ダブりを防ぐ）
};
//ros2ノード
class STS3215Node : public rclcpp::Node
{
public:
    STS3215Node() : Node("sts3215_node")
    {
        const auto port = declare_parameter<std::string>("port","/dev/ttyUSB0");
        //動作指令を送るかどうかの設定を取得
        motion_enabled_ = declare_parameter<bool>("enable_motion",false);
        //サーボIDの設定
        ids_ = declare_parameter<std::vector<int64_t>>("servo_ids",{1,2,3,4,5,6});
        //各関節が0radの時のサーボ位置を取得
        zeros_ = declare_parameter<std::vector<int64_t>>("zero_ticks", std::vector<int64_t>{});
        //関節角度の向き指定
        directions_ = declare_parameter<std::vector<int64_t>>("directions", std::vector<int64_t>{});
        //各関節で許可するサーボ位置の下限・上限を取得する
        min_ticks_ = declare_parameter<std::vector<int64_t>>("min_ticks",std::vector<int64_t>{});
        max_ticks_ = declare_parameter<std::vector<int64_t>>("max_ticks",std::vector<int64_t>{});
        //取得した配列の個数や値が妥当か検査する
        validateSettings();
        //ライブラリ側を、通常の書き込み命令に対する応答を待つ設定にする
        bus_.Level = 1;
        bus_.IOTimeOut = 20;
        if(!bus_.begin(1000000,port.c_str()))
        {
            throw std::runtime_error("Cannot open serial port: " + port);
        }
        for (std::size_t i=0;i<N;++i)
        {
            const int mode = bus_.readByte(id(i),SMS_STS_MODE);//関節iに対応するサーボの動作モードを1バイト読む
            checkResult(mode >= 0,i,"read mode");
            if (mode != 0)
            {
                throw std::runtime_error(names_[i] + ": position mode required");
            }
            if (motion_enabled_)
            {
                checkTorque(i);//動作指令を有効にする場合は、トルクがすでにonになっているか確認する
            }
        }
        readPositions();
        target_ = measured_;
        publisher_ = create_publisher<JoinstState>("/so100/real/joint_states", rclcpp::QoS(1));
        subscription_ = create_subscription<JoinstState>("/so100/real/joint_commands",rclcpp::QoS(1).reliable(),std::bind(&STS3215Node::onCommand,this,std::placeholders::_1));
        timer_ = create_wall_timer(20ms,std::bind(&STS3215Node::onTimer,this));
        RCLCPP_INFO(get_logger(),"STS3215 connected. Position commands: %s", motion_enabled_ ? "enabled" : "disabled");
    }

private:

    using JoinstState = sensor_msgs::msg::JointState;
    using Clock = std::chrono::steady_clock;
    static constexpr std::size_t N = 6;
    static constexpr double TICKS_PER_RAD = 4096.0 / (2.0 * 3.14159265358979323846);
    static constexpr double MAX_LEAD_RAD = 0.05;
    static constexpr uint16_t SERVO_SPEED = 200;
    static constexpr uint8_t SERVO_ACCELERATION = 10;

    const std::array<std::string,N> names_{
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper"
    };

    ServoBus bus_;

    std::vector<int64_t> ids_, zeros_, directions_;
    std::vector<int64_t> min_ticks_, max_ticks_;

    std::array<double, N> measured_{};
    std::array<double, N> target_{};

    bool motion_enabled_{false};
    bool have_command_{false};
    bool fault_{false};
    Clock::time_point last_command_{};

    rclcpp::Publisher<JoinstState>::SharedPtr publisher_;
    rclcpp::Subscription<JoinstState>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;


    uint8_t id(std::size_t i)const{
        return static_cast<uint8_t>(ids_[i]);
    }

    void validateSettings() const{
        if (ids_.size() != N ||
            zeros_.size() != N ||
            directions_.size() != N ||
            min_ticks_.size() != N ||
            max_ticks_.size() != N)
        {
            throw std::runtime_error("servo_ids, zero_ticks, directions, min_ticks and max_ticks "
        "must each contain 6 values");
        }
        
        for (std::size_t i = 0; i < N; ++i){
            if (ids_[i] < 1 || ids_[i] > 253 ||
                std::count(ids_.begin(), ids_.end(), ids_[i]) != 1 ||
                zeros_[i] < 0 || 
                zeros_[i] > 4095 ||
                (directions_[i] != 1 && directions_[i] != -1) ||
                min_ticks_[i] < 0 || 
                max_ticks_[i] > 4095 ||
                min_ticks_[i] >= max_ticks_[i]) 
            {
                throw std::runtime_error("Invalid calibration for " + names_[i]);
            }
        }
    }

    double angleToTicks(std::size_t i,double angle)const
    {
        return (zeros_[i] + directions_[i]*angle*TICKS_PER_RAD);
    }
    double ticksToAngle(std::size_t i,int ticks)const{
        return directions_[i]*(ticks - zeros_[i])/TICKS_PER_RAD;
    }

    //通信結果を検査する共通関数
    //success:呼び出しもとで判定した成功・失敗
    //i:関節番号、operation:エラー表示に使う操作名
    void checkResult(bool success, std::size_t i,const std::string & operation){
        if (!success ||
            bus_.getLastError() != 0 ||
            bus_.getState() != 0)
        {
            throw std::runtime_error( names_[i] + ": " + operation + ", communication=" + std::to_string(bus_.getLastError()) + ", status=" + std::to_string(bus_.getState()));
        }
        /*
        ・関数の結果が失敗
        ・ライブラリが通信エラーを報告
        ・サーボが異常状態を報告
        に該当すればエラー
        */
    }

    void checkTorque(std::size_t i){
        const int torque = bus_.readByte(id(i), SMS_STS_TORQUE_ENABLE);//指定サーボのトルク有効状態を読む
        checkResult(torque >= 0,i,"read torque");
        if (torque != 1){
            throw std::runtime_error(names_[i] + ": torque is not enabled");
        }
    }

    void readPositions()
    {
        std::array<double,N> next{};

        for (std::size_t i=0;i<N;++i)
        {
            const int received = bus_.FeedBack(id(i));//サーボから状態情報を読み取り、ライブラリ内部に保存する
            checkResult(received > 0,i,"feedback");
            const int ticks = bus_.ReadPos(-1);//直前のfeedbackで取得した情報から、位置を取り出す
            if (ticks < min_ticks_[i] ||
                ticks > max_ticks_[i])
            {
                throw std::runtime_error(names_[i] + ": measured position outside limits");
            }
            next[i] = ticksToAngle(i,ticks);
        }
        measured_ = next;//全関節の取得に成功した後に、実効値をまとめて更新する
    }

    void onCommand(const JoinstState::SharedPtr message)
    {
        if (!motion_enabled_ || fault_) return;
        if (message->name.size() != N || message->position.size() != N) return;

        std::array<double,N> next{};

        for (std::size_t i=0;i<N;++i)
        {   //受信した名前の配置から、対象の関節名を探す
            const auto found = std::find(message->name.begin(),message->name.end(),names_[i]);

            if (found == message->name.end() ||
                std::count(message->name.begin(),message->name.end(),names_[i]) != 1)
            {
                return;
            }
            //見つかった位置を、配列の添え字に変換する
            const auto index = std::distance(message->name.begin(),found);
            //その名前に対応する角度を取り出す
            const double angle = message->position[index];
            //NaNや無限大など、通常の有限関数でない値を拒否
            if(!std::isfinite(angle)) return;
            //サーボ位置へ変換
            const double ticks = angleToTicks(i,angle);
            if(ticks < min_ticks_[i] || ticks > max_ticks_[i]) return;
            next[i] = angle;
        }
        //全関節が有効だった場合だけ、目標角度を更新、受信記録を記録、有効な指令を受信済みとして記録する
        target_ = next;
        last_command_ = Clock::now();
        have_command_ = true;
    }

    //6関節分の角度を、参照で受け取る（constなので、関数内では入力配列を書き換えない）
    void writeTargets(const std::array<double,N>& angles)
    {
        for (std::size_t i=0;i<N;++i)
        {
            checkTorque(i);
        }
        for (std::size_t i=0;i<N;++i)
        {   //送る目標を、実測角度の±0.05rad以内に収める
            const double angle = std::clamp(angles[i],measured_[i]-MAX_LEAD_RAD,measured_[i]+MAX_LEAD_RAD);
            //実測0.2rad、目標1.0rad
            //radをサーボ位置へ変換（angleToTicks）→最も近い整数へ丸める（std::lround()）→int型へ変換（static_cast）
            const int ticks = static_cast<int>(std::lround(angleToTicks(i,angle)));
            if(ticks < min_ticks_[i] || ticks > max_ticks_[i])
            {
                throw std::runtime_error("Goal outside calibrated limits");
            }
            //サーボID、目標位置、速度、加速度を指定して送信する
            const int result = bus_.WritePosEx(id(i),static_cast<int16_t>(ticks),SERVO_SPEED,SERVO_ACCELERATION);
            checkResult(result == 1,i,"write position");
        }
    }

    void onTimer()
    {
        if(fault_) return;

        try
        {
            readPositions();
            JoinstState message;
            message.header.stamp = now();
            message.name.assign(names_.begin(),names_.end());
            message.position.assign(measured_.begin(),measured_.end());
            publisher_->publish(message);
            if(!motion_enabled_ || !have_command_) return;

            if (Clock::now() - last_command_ > 300ms)
            {
                writeTargets(measured_);
                fault_ = true;
                RCLCPP_ERROR(get_logger(),"Command timeout. Hold requested; restart required.");
                return;
            }
            writeTargets(target_);
        }
        catch (const std::exception & error)
        {
            fault_ = true;
            RCLCPP_ERROR(get_logger(),"Hardware fault: %s. Restart required.",error.what());
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc,argv);
    int result = 0;

    try
    {
        auto node = std::make_shared<STS3215Node>();
        rclcpp::spin(node);
    }
    catch (const std::exception & error)
    {
        RCLCPP_FATAL(rclcpp::get_logger("sts3215_node"), "%s", error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}