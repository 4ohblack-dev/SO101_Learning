#include "so100_control/joy_teleop.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>

namespace so100_control
{

JoyTeleop::JoyTeleop() : Node("so100_joy_teleop")//コンストラクタ（親クラスの、rclcpp::Nodeを初期化）
{//ros2パラメタを宣言して、値を取得する(値の型はstring)
  //topic名の宣言
  const auto joy_topic = declare_parameter<std::string>("joy_topic", "/joy");
  const auto state_topic = declare_parameter<std::string>("state_topic", "/so100/sim/joint_states");
  const auto command_topic = declare_parameter<std::string>("command_topic", "/so100/sim/joint_commands");
//操作設定用の宣言
  deadzone_ = declare_parameter<double>("deadzone", 0.15);//deadzoneの定義
  joy_timeout_sec_ = declare_parameter<double>("joy_timeout_sec", 0.3);
  state_timeout_sec_ = declare_parameter<double>("state_timeout_sec", 0.5);
  max_target_lead_rad_ = declare_parameter<double>("max_target_lead_rad", 0.1);
  speeds_ = declare_parameter<std::vector<double>>("speeds", std::vector<double>{0.3, 0.3, 0.3, 0.3, 0.3, 0.3});//6関節それぞれの操作速度
  directions_ = declare_parameter<std::vector<double>>("directions", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});//6関節それぞれの操作方向
/*
例外処理
throw → catch
今回は、mainの最後にcatchがある。
*/
  if (!std::isfinite(deadzone_) || deadzone_ < 0.0 || deadzone_ >= 1.0)
  {
    throw std::invalid_argument("deadzone must satisfy 0 <= deadzone < 1");
  }
  if (!std::isfinite(joy_timeout_sec_) ||
      !std::isfinite(state_timeout_sec_) ||
      !std::isfinite(max_target_lead_rad_) ||
      joy_timeout_sec_ <= 0.0 || state_timeout_sec_ <= 0.0 ||
      max_target_lead_rad_ <= 0.0)
  {
    throw std::invalid_argument("timeouts and target lead must be finite and positive");
  }
  //6関節分の設定を検査する
  if (speeds_.size() != kJointCount || directions_.size() != kJointCount)
  {
    throw std::invalid_argument("speeds and directions must contain 6 values");
  }
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    if (!std::isfinite(speeds_[i]) || speeds_[i] < 0.0)
    {
      throw std::invalid_argument("speeds must be finite and nonnegative");
    }
    if (directions_[i] != 1.0 && directions_[i] != -1.0)
    {
      throw std::invalid_argument("directions must contain only 1.0 or -1.0");
    }
  }
//ros2へメッセージを送信するpublisherを作る
  //送信先topic:command_topic、送るメッセージの型：jointstate、rclcpp::QoS(1)保持する履歴の深さを１にする
  command_publisher_ = create_publisher<JointState>(command_topic, rclcpp::QoS(1).reliable());
//subscriberを作る
  //joy_topic：受信するトピック()、sensordataQos()センサデータ向けの通信設定
  //メッセージがと祖いたら、このオブジェクトのonJoy()に、そのメッセージを渡して呼び出す
  joy_subscription_ = create_subscription<Joy>(joy_topic, rclcpp::SensorDataQoS(),std::bind(&JoyTeleop::onJoy, this, std::placeholders::_1));
  //onStateに渡して呼び出す
  state_subscription_ = create_subscription<JointState>(state_topic, rclcpp::SensorDataQoS(),std::bind(&JoyTeleop::onState, this, std::placeholders::_1));
//最初のタイマー処理で、経過時間を計算する基準。
  last_tick_time_ = Clock::now();
  //20msごとにonTimer()を呼ぶタイマーを作る
  timer_ = create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&JoyTeleop::onTimer, this));
  RCLCPP_INFO(get_logger(),
    "Waiting for joint states and /joy. Release L1 once, then hold L1 to move.");
}
// /joyが届いたときに呼ばれる関数
void JoyTeleop::onJoy(const Joy::SharedPtr message)//SharedPtr→利用者がいなくなったときに自動で解放される要管理されたポインタで、const はポインター変数自体の変更を制限している。
{
  // 現在のPS4JoyPublisherの配列配置に対応する。
  if (message->axes.size() < 8 || message->buttons.size() < 5)
  {
    have_joy_ = false;//有効な入力がない状態にする
    return;
  }
  std::array<double, kJointCount> new_inputs{};//6関節分の入力を保存する配列
  for (std::size_t i = 0; i < kJointCount; ++i)//6関節分を順番に処理
  {
    const double value = message->axes[axis_indices_[i]];//関節に割り当てた入力軸を取り出す
    if (!std::isfinite(value))
    {
      have_joy_ = false;
      return;
    }
    new_inputs[i] = std::clamp(value, -1.0, 1.0);//入力を-1~1に収める
  }
  inputs_ = new_inputs;//全関節分の検査に成功したので、入力を保存
  enable_pressed_ = message->buttons[4] != 0;//L1が押されているかの確認
  last_joy_time_ = Clock::now();//有効な入力を受信した時刻を保存する
  have_joy_ = true;//有効な入力を受け取った状態にする
}
//u\Unityから現在の関節状態が届いたときに呼ばれる
void JoyTeleop::onState(const JointState::SharedPtr message)
{
  if (message->name.size() != message->position.size())//関節名と角度の個数が一致するかの確認
  {
    return;
  }
  std::array<double, kJointCount> positions{};
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    //受信した名前の配列から、目的の関節名を探す
    //find(配列の先頭、配列の最後の要素の、さらに次の位置、探したい関節名)
    const auto found = std::find(message->name.begin(), message->name.end(), joint_names_[i]);
    if (found == message->name.end() ||
        std::count(message->name.begin(), message->name.end(), joint_names_[i]) != 1)
    {//関節名が見つからない or関節名がダブっている
      return;
    }
    //見つかった位置を、配列番号へ変換
    const auto index = static_cast<std::size_t>(found - message->name.begin());
    const double position = message->position[index];//見つけた関節名と同じ番号の角度を取得
    if (!std::isfinite(position))
    {
      return;
    }
    positions[i] = position;//自分の関節順に並べなおして保存する
  }
  current_positions_ = positions;
  last_state_time_ = Clock::now();
  if (!have_state_)//まだ一度も有効な状態を受け取っていないかの確認
  {
    // 起動時にゼロ姿勢へ動かさず、実際の角度から開始する。
    targets_ = current_positions_;//最初の目標角度を現在の角度に合わせる
    RCLCPP_INFO(get_logger(), "Joint states received. Initial targets set.");
  }
  have_state_ = true;//「有効な関節状態を受信済み」の状態にする
}
//deadzoneより外側の入力を、0~1に変換してから元の符号をつける。
double JoyTeleop::applyDeadzone(double value) const
{
  const double magnitude = std::abs(value);
  if (magnitude <= deadzone_)
  {
    return 0.0;
  }
  return std::copysign((magnitude - deadzone_) / (1.0 - deadzone_), value);
}

void JoyTeleop::onTimer()
{
  const auto current_time = Clock::now();
  //前回のタイマー処理から何秒経ったか計算する
  //duration<double>：秒単位の小数として扱う、()時間差、.count()数値を取り出す
  double dt = std::chrono::duration<double>(current_time - last_tick_time_).count();
  last_tick_time_ = current_time;
  dt = std::clamp(dt, 0.0, 0.05);

  //関節状態が操作に使えるか判定する（有効な状態を一度は受信した and 最後の受信からタイムアウト時間以内）
  const bool state_fresh = have_state_ &&
    std::chrono::duration<double>(current_time - last_state_time_).count() <=
    state_timeout_sec_;
  //コントローラ入力にも同じ判定をする
  const bool joy_fresh = have_joy_ &&
    std::chrono::duration<double>(current_time - last_joy_time_).count() <=
    joy_timeout_sec_;
/*
enable_pressed_ → 最新の入力ではL1が押されているか
armed_ → L1を話した状態を確認し、操作開始できる状態か
L1を押している間だけアームを操作できるようにする設定。
active_ → 操作モードに入っているか
*/
  if (!state_fresh)//関節が未受信 or タイムアウト
  {
    if (active_)//それまで捜査中
    {
      RCLCPP_WARN(get_logger(), "Joint state timeout. Release L1 after recovery.");
    }
    armed_ = false;//操作開始の許可状態を解除
    active_ = false;//操作モードを終了する
    return;
  }
  if (!joy_fresh)//関節状態が古い
  {
    if (active_)//操作中だった場合
    {
      targets_ = current_positions_;//保持する目標を、最後に受信した現在角度へ変更する
      RCLCPP_WARN(get_logger(), "Joy timeout. Holding latest measured position.");
    }
    armed_ = false;
    active_ = false;
    publishTargets();
    //操作状態を解除、保持用の目標角度を送る
    return;//returnで、目標角度を増減させる処理へは進まない。
  }
  if (!enable_pressed_)//L1が押されていない状態
  {
    if (active_ || !armed_)
    {
      targets_ = current_positions_;
    }
    active_ = false;
    armed_ = true;
    publishTargets();
    //操作状態を解除、L1が離されている状態を確認できた
    return;
  }
  // 起動・通信復帰後は、L1を一度離してから再操作する。
  if (!armed_)
  {
    return;
  }
  if (!active_)//これから新しく操作モードに入る
  {
    targets_ = current_positions_;//実際の現在角度から操作を始める
    active_ = true;//操作モードに入ったことを記録
  }
  for (std::size_t i = 0; i < kJointCount; ++i)//6関節について計算
  {
    const double input = applyDeadzone(inputs_[i]) * directions_[i];//deadzoneで中央値付近の値を0にする and 必要なら向きを反転する
    targets_[i] += input * speeds_[i] * dt;//新しい目標角度 = 前の目標角度 + 入力 x 操作速度 x 経過時間
    //スティック入力をそのまま角度に対応させず、角度を変える速さとして使う。
    targets_[i] = std::clamp(targets_[i],
      current_positions_[i] - max_target_lead_rad_,
      current_positions_[i] + max_target_lead_rad_);//目標が現在角度から離れすぎないように制限する
    targets_[i] = std::clamp(targets_[i], lower_limits_[i], upper_limits_[i]);//URDFの可動範囲に合うようにclamp
  }
  //現在の動きから目標が離れすぎないようにする
  //関節の可動範囲を超える目標を出さない
  publishTargets();//6関節分の目標角度を送信する
}

void JoyTeleop::publishTargets()
{
  JointState message;
  message.header.stamp = now();
  message.name.assign(joint_names_.begin(), joint_names_.end());//
  message.position.resize(kJointCount);
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    message.position[i] = std::clamp(targets_[i], lower_limits_[i], upper_limits_[i]);//送信直前にも、関節可動範囲に収める処理
  }
  command_publisher_->publish(message);//messageを配信
}

}  // namespace so100_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    // 単一スレッドで購読とタイマーを処理する。
    rclcpp::spin(std::make_shared<so100_control::JoyTeleop>());
  }
  catch (const std::exception & error)//標準例外を受け取る
  {
    RCLCPP_FATAL(rclcpp::get_logger("so100_joy_teleop"), "%s", error.what());
    rclcpp::shutdown();
    return 1;//異常終了を表す終了コード1を返す
  }
  rclcpp::shutdown();
  return 0;
}

/*
main()
  ↓
ROS 2の初期化
  ↓
JoyTeleopのコンストラクター
  ├─ 設定を読む
  ├─ 設定を検査する
  ├─ 送受信を登録する
  └─ タイマーを登録する
  ↓
spin()
  ├─ 入力受信      → onJoy()
  ├─ 関節状態受信  → onState()
  └─ 約20msごと    → onTimer()
                        ↓
                    publishTargets()


inputs_               → コントローラー入力 
current_positions_    → Unityで実際に動いた関節角度 
targets_              → Unityへ指令したい目標角度 

*/