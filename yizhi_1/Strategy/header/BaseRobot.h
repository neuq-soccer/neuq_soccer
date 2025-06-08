#include "platform.h"
#include "string"

typedef struct PID
{
	double proportion; //比例常数
	double integral; //积分常数 
	double derivative; //微分常数
	int lastError; 
	int Error;
	double i_out;
	double d_out;
	double p_out;
	double current;
	double total_out;
	double i_max;
	double Target;
	double i_Band;
} PID;

/*
机器人基类
维护机器人信息和基本动作（跑、转、甩）
子类建议只在基本动作基本上封装简单动作，更复杂的动作作为单人策略
信息复杂计算使用计算类
*/

class BaseRobot
{
public:
	BaseRobot();
	virtual ~BaseRobot();

	void moveTo(double tarX, double tarY);	// 跑位函数，跑位到(tarX,tarY)

	void update(Simuro::Robot* robot);	// 将平台提供的环境信息赋给BaseRobot，每拍更新

	void saveLastInformation(double footBallNow_X, double footBallNow_Y);	// 保存机器人本拍信息，留作下一拍使用，可用于拓展保存更多信息

	Simuro::Vector2 getPos();	// 获取机器人位置

	Simuro::Vector2 getLastPos();	// 获取机器人上一拍位置

	Simuro::Vector2 getLastPos_P();	// 获取基于预测的机器人上一拍位置

	Simuro::Vector2 getLastBallPos();	// 获取球上一拍位置

	double getRotation();	// 获取机器人旋转角

	double getLastRotation();	// 获取上一拍机器人旋转角

	double getLeftWheelVelocity();	// 获取机器人左轮速

	double getRightWheelVelocity();	// 获取机器人右轮速

	double getTarX();	// 获取机器人跑位目标点x坐标
	
	double getTarY();	// 获取机器人跑位目标点y坐标

	void Velocity(double vl, double vr);	// 直接赋左右轮速

	void TurnAngle(double Cur_Angle, double Tar_Angle); // 测试用的转角函数

	double Myfabs(double Temp); //写多了，绝对值函数，懒得删

	void moveto_within_x_limits(double x_limits, double tar_x, double tar_y);	// 机器人进行跑位，但机器人x坐标不能超过x_limits。

	void Move_Go(double Tar_x, double Tar_y);

	void Move_GoTar(double tar_x, double tar_y);//用这个函数定点, 别用，用了只能碰一次球就寄了

	void throwBall(double ballx, double bally);

	void moveWithAngle(double tarX, double tarY, double tar_angle);

	void turnToAngle(double tar_angle);

	void shoot(double ballx, double bally);

	void breakThrough(BaseRobot oppRobots[5], double tarx, double tary);

	void keepYPushBall(double keepY1, double keepY2, double footBallNow_X, double footBallNow_Y);

	void PredictRobotInformation(int tick_delay);	// 预测自己tick_delay拍后的位置和状态

	Simuro::Robot GetRobotInformation(int time);	// 获得机器人近time拍的状态，time < 0获得历史状态，time > 0获得预测状态

	double calculate_nextNtick_displace_125(double speed, int tickNum);

	void calculate_tangen(double tarx_ball, double tary_ball, double posx, double posy, double ballx, double bally);

	void shoot_with_angle(double tarx_ball, double tary_ball, double posx, double posy, double ballx, double bally);

private:
	Simuro::Robot* robot = nullptr;
	double lastU = 0;	// pid控制变量U
	double lastU1 = 0;	// pid控制变量U1
	double lastTargetX = 0, lastTargetY = 0;	// 机器人上一拍目标点
	double lastRobotX = 0, lastRobotY = 0;	// 机器人上一拍位置
	double lastRobotX_P = 0, lastRobotY_P = 0;	// 基于预测的机器人上一拍位置
	double lastBallX = 0, lastBallY = 0;	// 上一拍位置
	double lastRotation = 0;	// 机器人上一拍旋转角
	double tangentX, tangentY, circle_center_x, circle_center_y, ball_future4_x, ball_future4_y, out_circle_y, out_circle_x, d_pos_lcircle, d_pos_rcircle;

	PID* sptr;
	PID* x_dis;
	void initPid();	//初始化PID参数
	void pidCal(PID* pid, double nowPoint, double tarPoint);
public:
	Simuro::Robot HistoryInformation[8] = { {{0,0},0,{0,0}} };
	Simuro::Robot PredictInformation[8] = { {{0,0},0,{0,0}} };
	double AngularSpeed[8] = { 0 };
	bool reset = false;	// 如果机器人重新摆位，则需要重置
};

class DataLoader
{
public:
	int get_event(int tick);//获得tick时刻的比赛状态
	void set_tick_state(int tick, int event_state);//设置此时的信息
private:
	int tick;	// 定义当前所在的拍数
	int event_states[100000];	// 用于存储事件状态

};