#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <time.h>
#include<fstream>
struct node {
  double x, y, z, ang_change, ang_base;
};
// create the TeleopTurtle class and define the joyCallback function that will take a joy msg
class TeleopTurtle
{
public:
  std::string LoadPath;
  std::string SavePath;
  TeleopTurtle();
  void sim_pub_v1(std::vector<double> &vel, std::vector<double> &pos, int &cnt);
  void sim_pub_v2(std::vector<double> &pos, int &cnt);
  bool sim_pub_p1(std::vector<node> &pos, int &cnt);
  bool sim_pub_p2(std::vector<node> &pos, int &cnt);
  void sim_pub(std::vector<node> &pos, int &cnt);
  void time_creator(std::vector<node> &target, std::vector<double> &pos);
  double self_control(double setpoints, double current_state);
  //double self_control2(std::vector<double> &setpoints, int cnt);
private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void sim_sub_v(const geometry_msgs::Twist::ConstPtr& twist);
  void sim_sub_pos(const geometry_msgs::Twist::ConstPtr& twist);
  
  ros::NodeHandle nh_;

  int linear_, angular_, cnt;   // used to define which axes of the joystick will control our turtle
  double l_scale_, a_scale_;
  double change_now;
  bool isFirst, flag1, flag2;
  node pos_now, pos_first;
  std::vector<double> setpoints;
  ros::Publisher vel_pub_;
  ros::Publisher pos_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber change_sub_;
  ros::Subscriber pos_sub_;
  ros::Timer timer, timer2;
};
class PIDController {
public:
    PIDController(double p_gain, double i_gain, double d_gain) 
        : p_gain(p_gain), i_gain(i_gain), d_gain(d_gain), prev_error(0.0), integral(0.0) {}

    double control(double setpoint, double current_pos) {
        double error = setpoint - current_pos;
        integral += error;
        double derivative = error - prev_error;
        double output = p_gain * error + i_gain * integral + d_gain * derivative;
        prev_error = error;
        return output;
    }

private:
    double p_gain, i_gain, d_gain;
    double prev_error, integral;
};
TeleopTurtle::TeleopTurtle(): linear_(1), angular_(2),LoadPath("/home/ru/model/test_left.txt"), SavePath("/home/ru/model/test_left.txt")
{
  isFirst = 1;
  flag1 = 0,flag2 = 0;
  //  initialize some parameters
  nh_.param("axis_linear", linear_, linear_);  
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  nh_.param("LoadPath", LoadPath, LoadPath);
  nh_.param("SavePath", SavePath, SavePath);
  // create a publisher that will advertise on the command_velocity topic of the turtle

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  pos_pub_ = nh_.advertise<geometry_msgs::Twist>("/flipper_target_pos", 1);
  //subscribe to the joystick topic for the input to drive the turtle
  
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &TeleopTurtle::joyCallback, this);
  //change_sub_ = nh_.subscribe<geometry_msgs::Twist>("/flipper_target_v", 10, &TeleopTurtle::sim_sub_pos, this);
  pos_sub_ = nh_.subscribe<geometry_msgs::Twist>("/Pos_target", 10, &TeleopTurtle::sim_sub_pos, this);
}

float ToDegree(float radians) {  
    return radians * 180.0 / M_PI;  
}
void TeleopTurtle::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist twist;
  geometry_msgs::Twist twist_pos;

  // take the data from the joystick and manipulate it by scaling it and using independent axes to control the linear and angular velocities of the turtle
  twist.angular.z = a_scale_*joy->axes[angular_] / 2;
  twist.linear.x = l_scale_*joy->axes[linear_] / 2;
  vel_pub_.publish(twist);
  
  if (joy->buttons[3]) {//y
    twist_pos.linear.x = 5;
    twist_pos.angular.z = 5;
  }
  if (joy->buttons[0]) {//a
    twist_pos.linear.x = -5;
    twist_pos.angular.z = -5;
  }
  if (joy->buttons[1]&& joy->buttons[2] != 0) {//b
    twist_pos.linear.x  = 0;
    twist_pos.angular.z = 0;
  }

  pos_pub_.publish(twist_pos); 
}
void TeleopTurtle::time_creator(std::vector<node> &target, std::vector<double> &pos) {
  cnt = 0;
  //timer = nh_.createTimer(ros::Duration(0.005), boost::bind(&TeleopTurtle::sim_pub_v2, this,  pos, cnt));
  timer2 = nh_.createTimer(ros::Duration(0.005), boost::bind(&TeleopTurtle::sim_pub, this, target, cnt));

}

void ReadFile2(std::string filename, std::vector <node> &temp_pos) {
  double x, z, ang_base, ang_change;
  std::ifstream in;//define 'in' as the way to input 
  in.open(filename,std::ios::in);//open the file
  while(in >> x >> z >> ang_base >> ang_change) {
    node temp;
    temp.x = x;
    temp.y = -1;
    temp.z = z;
    temp.ang_base = ang_base;
    temp.ang_change = ang_change;
    temp_pos.push_back(temp);
    std::cout << temp.x << " " << temp.y << " " << temp.z << " " << temp.ang_base << " " << temp.ang_change << std::endl;
  } 
  in.close();
  return;
}
void WriteFile2(std::string filename, node temp) {
  std::ofstream myfile;
  myfile.open(filename,std::ios::app);
  myfile << temp.x << ' ' << temp.z << " " << temp.ang_base << " " << temp.ang_change << "\n";
  myfile.close();
}
double TeleopTurtle::self_control(double setpoint,double current_state) {
  double p_gain = 0.1, i_gain = 0.01, d_gain = 0.001;
  PIDController controller(p_gain, i_gain, d_gain);
  // 期望的摆臂位置序列
  // 假设当前摆臂位置为current_pos
  // 计算控制输出
  double control_output = controller.control(setpoint, current_state);
  // 将控制输出应用到摆臂applyControl(control_output);
  return control_output;
}

bool TeleopTurtle::sim_pub_p1(std::vector<node> &set_pos, int &cnt) {
  geometry_msgs::Twist twist_pos_vel;   
  //std::cout << "si zai zhe le?" << std::endl;
  // pos_pub_.publish(twist_pos);
  double k = 1;
  node temp = this->pos_now;//当前的位置信息
    bool isOK_pos = 0;
    if (abs(set_pos[cnt].x - temp.x) >= 0.01) {
      double new_v = self_control(set_pos[cnt].x, temp.x);
      if (new_v <= 0.1 && new_v > 0) new_v = 0.1;
      if (new_v >= -0.1 && new_v < 0) new_v = -0.1; 
      twist_pos_vel.linear.x = k * new_v * -1;
      vel_pub_.publish(twist_pos_vel);
      isOK_pos = 1;
      return 0;
    }
    else {
      twist_pos_vel.linear.x = 0;
      twist_pos_vel.angular.z = 0;
      vel_pub_.publish(twist_pos_vel);
      return 1;
    }
}
bool TeleopTurtle::sim_pub_p2(std::vector<node> &set_pos, int &cnt) {
  geometry_msgs::Twist twist_pos_change;  
  //std::cout << "si zai zhe le?" << std::endl;
  // pos_pub_.publish(twist_pos);
  double k = 1;
  node temp = this->pos_now;//当前的位置信息
    if (abs(set_pos[cnt].ang_change - temp.ang_change) >= 0.1) {
      double new_v = self_control(set_pos[cnt].ang_change, temp.ang_change);
      twist_pos_change.angular.z = k * new_v * -1;
      pos_pub_.publish(twist_pos_change);
      return 0;
    }
    else {
          twist_pos_change.angular.z = 0;
    pos_pub_.publish(twist_pos_change);

      return 1;
    }
}
void TeleopTurtle :: sim_pub(std::vector<node> &set_pos, int &cnt) {
  geometry_msgs::Twist twist_pos_vel; 
  geometry_msgs::Twist twist_pos_change;  

  if (cnt < set_pos.size()){
    std::cout << this->pos_now.x << " " << set_pos[cnt].x << " " << this->pos_now.ang_change << " " << set_pos[cnt].ang_change << std::endl;
    
    // flag1 = sim_pub_p1(set_pos, cnt);
    std::cout << flag1 << std::endl;
    if (flag1)
      flag2 = sim_pub_p2(set_pos, cnt);
    else
      flag1 = sim_pub_p1(set_pos, cnt);
    if (flag1 && flag2)
    {
      flag1 = 0,flag2 = 0;
      cnt++;
    }
      
  }
  else {
    twist_pos_change.angular.z = 0;
    pos_pub_.publish(twist_pos_change);
    twist_pos_vel.linear.x = 0;
    twist_pos_vel.angular.z = 0;
    vel_pub_.publish(twist_pos_vel);
    
    
  }
}

void TeleopTurtle::sim_sub_pos(const geometry_msgs::Twist::ConstPtr& twist) {
  /*linear = position, 
  angular = {x = ang_change.x, y = ang_base.x, z = 0 */
  node temp_now;
  temp_now.x = twist->linear.x;
  temp_now.y = twist->linear.y;
  temp_now.z = twist->linear.z;
  temp_now.ang_base = ToDegree(twist->angular.y);
  temp_now.ang_change = ToDegree(twist->angular.x);
  this->pos_now = temp_now;
  std::cout << "now pos  is " << temp_now.ang_change << std::endl;
  if (isFirst == true) {
    isFirst = false;
    this->pos_first = temp_now;
  }
  this->pos_now.x -= this->pos_first.x;
  this->pos_now.y -= this->pos_first.y;
  this->pos_now.z -= this->pos_first.z;
  
  WriteFile2(SavePath, temp_now);
}

int main(int argc, char** argv) {
  // initialize our ROS node, create a teleop_turtle, and spin our node until Ctrl-C is pressed
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;
  std::vector<node> target;
  ReadFile2("/home/ru/model/pos.txt", target);
  
  // std::vector<double> vel;
   std::vector<double> pos;
  // ReadFile(teleop_turtle.LoadPath,vel, pos);
  // ros::Duration(1).sleep();
  teleop_turtle.time_creator(target, pos);
  ros::spin();
  
}
