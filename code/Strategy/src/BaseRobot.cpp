#include "../header/BaseRobot.h"

#include <cmath>

#include "../header/constant.h"
#include "../header/globalVariable.h"
#include "fstream"
#include "iostream"

// 构造函数
BaseRobot::BaseRobot()
{
	sptr = new PID;
	x_dis = new PID;
	initPid();
	//initpose();
}

// 析构函数
BaseRobot::~BaseRobot()
{
	delete sptr;
	delete x_dis;
}

// 用于更新BaseRobot数据，因为BaseRobot是我们自己的定义的类型，平台每拍只会给Robot类型传数据，所以需要我们自己把Robot类的数据传给BaseRobot
void BaseRobot::update(Simuro::Robot* robot)
{
	this->robot = robot;
}

// 用于读取当前状态的位置
Simuro::Vector2 BaseRobot::getPos()
{
	return robot->position;
}

// 用于读取上一拍位置
Simuro::Vector2 BaseRobot::getLastPos()
{
	Simuro::Vector2 lastPos;
	lastPos.x = lastRobotX;
	lastPos.y = lastRobotY;
	return lastPos;
}

Simuro::Vector2 BaseRobot::getLastPos_P()
{
	Simuro::Vector2 lastPos;
	lastPos.x = lastRobotX_P;
	lastPos.y = lastRobotY_P;
	return lastPos;
}

// 用于读取球的上一拍位置
Simuro::Vector2 BaseRobot::getLastBallPos()
{
	Simuro::Vector2 lastPos;
	lastPos.x = lastBallX;
	lastPos.y = lastBallY;
	return lastPos;
}

// 用于获取机器人旋转角
double BaseRobot::getRotation()
{
	return robot->rotation;
}

double BaseRobot::getLastRotation()
{
	return lastRotation;
}

// 用于获取机器人左轮速
double BaseRobot::getLeftWheelVelocity()
{
	return robot->wheel.leftSpeed;
}

// 用于获取机器人右轮速
double BaseRobot::getRightWheelVelocity()
{
	return robot->wheel.rightSpeed;
}

// 获取机器人跑位目标点x坐标
double BaseRobot::getTarX()
{
	return lastTargetX;
}

// 获取机器人跑位目标点y坐标
double BaseRobot::getTarY()
{
	return lastTargetY;
}

// 初始化pid参数
void BaseRobot::initPid()
{
	sptr->lastError = 0;
	sptr->Error = 0;
	sptr->proportion = 0.1;
	sptr->integral = 0.0001;
	sptr->derivative = 1.3;
	sptr->i_Band = 2;
	sptr->i_max = 4;
	sptr->Target = 0;
	sptr->current = 0;
	sptr->i_out = 0;
	sptr->d_out = 0;
	sptr->p_out = 0;
	sptr->total_out = 0;

	x_dis->lastError = 0;
	x_dis->Error = 0;
	x_dis->proportion = 3.30;
	x_dis->integral = 0;
	x_dis->derivative = 10.30;
	x_dis->i_Band = 20;
	x_dis->i_max = 50;
	x_dis->Target = 0;
	x_dis->current = 0;
	x_dis->i_out = 0;
	x_dis->d_out = 0;
	x_dis->p_out = 0;
	x_dis->total_out = 0;
}

double BaseRobot::Myfabs(double Temp)
{
	if (Temp < 0)
	{
		return -Temp;
	}
	else
	{
		return Temp;
	}
}

//赋轮速
void BaseRobot::Velocity(double vl, double vr) {

	robot->wheel.leftSpeed = vl;
	robot->wheel.rightSpeed = vr;
}

// 重载运算符 - ，用于位置间的减法，直接计算出距离
double operator-(Simuro::Vector2 a, Simuro::Vector2 b)
{
	double dx = a.x - b.x;
	double dy = a.y - b.y;
	double distance = sqrt(dx * dx + dy * dy);
	return distance;
}

Simuro::Robot BaseRobot::GetRobotInformation(int time)
{
	if (time >= 0)
	{
		return PredictInformation[time];
	}
	else
	{
		return HistoryInformation[-time];
	}
}

//pid计算
void BaseRobot::pidCal(PID *pid ,double nowPoint, double tarPoint)
{
	float dt = 0.05;
	pid->Target = tarPoint;
	pid->current = nowPoint;

	pid->Error = pid->Target - pid->current;
	pid->p_out = pid->proportion * pid->Error;
	if (Myfabs(pid->Error) < pid->i_Band)
	{
		pid->i_out += pid->integral * pid->Error * dt;

		if (pid->i_out > pid->i_max)
		{
			pid->i_out = pid->i_max;
		}
		else if (pid->i_out < -pid->i_max)
		{
			pid->i_out = -pid->i_max;
		}
	}
	pid->d_out = pid->derivative * (pid->Error - pid->lastError);
	pid->total_out = pid->p_out + pid->i_out + pid->d_out;
	pid->lastError = pid->Error;
}

// 方法2使用PID进行调参
// 这里用pid方法进行底层运动函数控制，具体pid算法请参考免费课程，如果想优化底层可以重新写此函数。
void BaseRobot::moveTo(double x, double y)
{
	if (tick == 1)//控制pid计算频率
	{
		initPid();
		lastU = 0;
		lastU1 = 0;
	}
	//if (tick == 1)
	//{
	//	initPid();
	//}
	double parameter = 0.5;//参数，调整速度用
	double vMax = 90; //最大速度,方便后边用sigmoid函数调整速度大小
	double vL, vR;
	double angleTo = 0, angleDiff = 0; //angleTo:足球机器人和小球的角度
	double dX = x - robot->position.x;
	double dY = y - robot->position.y;
	double dR2RX = robot->position.x - lastRobotX;
	double dR2RY = robot->position.y - lastRobotY;
	angleTo = (180.0 / M_PI) * atan2(dY,dX);
	double distance = sqrt(dX * dX + dY * dY);
	angleDiff = robot->rotation - angleTo;
	//正则化夹角
	while (angleDiff > 180) angleDiff -= 360;
	while (angleDiff < -180) angleDiff += 360;
	if (fabs(angleDiff) < 85)
	{
		//lastU = lastU + pidCal(robot->rotation, angleTo); //lastU是上一次的控制量
		pidCal(sptr, robot->rotation, angleDiff);
		lastU = lastU + sptr->total_out;
		vR = vMax + lastU;
		vL = vMax - lastU;
		Velocity(vL, vR);
	}
	else if (fabs(angleDiff) >= 90)
	{
		angleDiff += 180;
		if (angleDiff > 180)
		{
			angleDiff -= 360;
		}
		//lastU = lastU + pidCal(robot->rotation, angleTo);
		pidCal(sptr, robot->rotation, angleDiff);
		lastU = lastU + sptr->total_out;
		vR = -vMax + lastU;
		vL = -vMax - lastU;
		Velocity(vL, vR);
	}
	else if(angleDiff == 0)
	{
		vR = 0;
		vL = 0;
		Velocity(vL, vR);
	}
}

void BaseRobot::TurnAngle(double Cur_Angle, double Tar_Angle)
{
	if (tick == 1)
	{
		initPid();
		//initpose();
	}
	double speedr, speedl;
	double angleDiff = Tar_Angle - Cur_Angle;
	while (angleDiff > 180) angleDiff -= 360;
	while (angleDiff < -180) angleDiff += 360;

	if (fabs(angleDiff) < 90)
	{
		pidCal(sptr, robot->rotation, Tar_Angle);
		speedr = sptr->total_out;
		speedl = -sptr->total_out;
	}
	else if (fabs(angleDiff) >= 90)
	{
		angleDiff += 180;
		if (angleDiff > 180)
			angleDiff -= 360;
		pidCal(sptr, robot->rotation, Tar_Angle);
		speedr = sptr->total_out;
		speedl = -sptr->total_out;
	}
	else
	{
		speedr = 80;
		speedl = -80;
	}
	Velocity(speedl, speedr);
}

double speedr = 0, speedl = 0, speed_tr = 0, speed_tl = 0;
void BaseRobot::Move_Go(double Tar_x, double Tar_y)
{
	if (tick == 1 || tick == 2 || tick % 40 == 0)
	{
		initPid();
	}	
	double tar_angle = 0;
	double delta_x = Tar_x - robot->position.x;
	double delta_y = Tar_y - robot->position.y;
	tar_angle = atan2(delta_y, delta_x) * (180.0 / M_PI);
	while (tar_angle > 185) tar_angle -= 360;
	while (tar_angle < -185) tar_angle += 360;//限幅
	double Tar_distance = sqrt(delta_x * delta_x + delta_y * delta_y);
	double angle_diff = tar_angle - robot->rotation;
	//double vel = Tar_distance * Kp;
	pidCal(x_dis, Tar_distance, 0);
	pidCal(sptr, robot->rotation, tar_angle);
	double vel = -x_dis->total_out;
	speed_tl = sptr->total_out;
	speed_tr = sptr->total_out;
	if (fabs(angle_diff) < 85)
	{
		double omg1 = (tar_angle - robot->rotation) * 0.3;
		speedl = vel - speed_tl * 2.4 - omg1 * 0.1;
		speedr = vel + speed_tr * 2.4 + omg1 * 0.1;
	}
	else if (fabs(angle_diff) >= 90)
	{
		angle_diff += 180;
		if (angle_diff > 180)
			angle_diff -= 360;
		double omg2 = (tar_angle - robot->rotation) * 0.3;
		speedl = -vel - speed_tl * 2.4 - omg2 * 0.1;
		speedr = -vel + speed_tr * 2.4 + omg2 * 0.1;
	}
	else
	{
		speedl = -80;
		speedr = 80;
	}
	Velocity(speedl, speedr);
}

void BaseRobot::Move_GoTar(double tar_x, double tar_y)
{
	Move_Go(-tar_x, -tar_y);
}

// 限制机器人跑位坐标点不会超过x_limits
void BaseRobot::moveto_within_x_limits(double x_limits, double tar_x, double tar_y)
{
	if (this->getPos().x > x_limits)	// 若现在位置超过x_limits
	{
		this->Move_Go(x_limits, tar_y);	// 跑回限制界里面
	}
	else
	{
		if (tar_x > x_limits)	// 如果跑位目标点超过x_limits
		{
			this->Move_Go(x_limits, tar_y);	// 改变跑位目标点，往限制界里面跑
		}
		else
		{
			this->Move_Go(tar_x, tar_y);	// 否则正常运行
		}
	}
}

void BaseRobot::throwBall(double ballx, double bally)
{
	double dX = ballx - PredictInformation[tick_delay].position.x;
	double dY = bally - PredictInformation[tick_delay].position.y;
	double distance = sqrt(dX * dX + dY * dY);
	if (distance > 10 || dX < 0)
	{
		moveWithAngle(ballx - 1, bally, 0);
	}
	else
	{
		if (bally < 0)
			Velocity(125, -125);
		else
			Velocity(-125, 125);
	}
}

void BaseRobot::moveWithAngle(double tarX, double tarY, double tar_angle)	// 机器人跑位到某点，并固定在某个角度
{
	double dX = tarX - PredictInformation[tick_delay].position.x;
	double dY = tarY - PredictInformation[tick_delay].position.y;
	double distance = sqrt(dX * dX + dY * dY);
	if (distance > 1)
	{
		moveTo(tarX, tarY);
	}
	else
	{
		turnToAngle(tar_angle);
	}
}

void BaseRobot::turnToAngle(double tar_angle)
{
	if (tick == 1)
	{
		initPid();
		//initpose();
	}
	double speedr, speedl;
	double angleDiff = tar_angle - PredictInformation[tick_delay].rotation;
	while (angleDiff > 180) angleDiff -= 360;
	while (angleDiff < -180) angleDiff += 360;

	if (fabs(angleDiff) < 90)
	{
		pidCal(sptr, PredictInformation[tick_delay].rotation, tar_angle);
		speedr = sptr->total_out;
		speedl = -sptr->total_out;
	}
	else if (fabs(angleDiff) >= 90)
	{
		angleDiff += 180;
		if (angleDiff > 180)
			angleDiff -= 360;
		pidCal(sptr, PredictInformation[tick_delay].rotation, tar_angle);
		speedr = sptr->total_out;
		speedl = -sptr->total_out;
	}
	else
	{
		speedr = 80;
		speedl = -80;
	}
	Velocity(speedl, speedr);
}

void BaseRobot::shoot(double ballx, double bally)
{
	double dX = ballx - PredictInformation[tick_delay].position.x;
	double dY = bally - PredictInformation[tick_delay].position.y;
	if (ballx < -75)
	{
		Move_Go(-60, bally);
	}
	else if (ballx < 72.5)	// 如果不在射门范围内，先跑到射门点
	{
		Move_Go(ballx, bally);
	}
	else	// 在射门范围内调整位置射门。
	{
		throwBall(ballx, bally);
	}
}

void BaseRobot::breakThrough(BaseRobot oppRobots[5], double tarx, double tary)
{
	Move_Go(tarx, tary);
	double future_time = 14;	// 以匀速直线运动预测14拍之后
	double len = 5;	//碰撞检测用到的参数，表示长度。
	double deltay = 4;	// 增大此值可使碰撞减少更明显，但也会使机器人更敏感，更漂浮不定
	for (int i = 0; i < 5; i++)
	{
		double futureOppY = future_time * oppRobots[i].getPos().y - (future_time - 1) * oppRobots[i].getLastPos().y;	// 未来对方0号机器人坐标x
		if (oppRobots[i].getPos().x > robot->position.x && oppRobots[i].getPos().x < tarx
			|| oppRobots[i].getPos().x < robot->position.x && oppRobots[i].getPos().x > tarx)	// 如果我方0号与对方0号x的关系存在产生碰撞可能
		{
			if (robot->position.y > 0)	// 如果在上半场
			{
				if (futureOppY + len > robot->position.y)	// 如果我方0号与对方0号y的关系存在产生碰撞可能
				{
					Move_Go(tarx, tary + deltay);	// 转换目标点
				}
			}
			if (robot->position.y < 0)	// 如果在下半场
			{
				if (futureOppY - len < robot->position.y)	// 如果我方0号与对方0号y的关系存在产生碰撞可能
				{
					Move_Go(tarx, tary - deltay);	// 转换目标点
				}
			}
		}
	}
}

void BaseRobot::keepYPushBall(double keepY1, double keepY2, double footBallNow_X, double footBallNow_Y)
{
	if (getPos().x < keepY1)
	{
		Move_Go(keepY1, footBallNow_Y);
	}
	else
	{
		if (footBallNow_X < keepY1)
		{
			Move_Go(keepY1, footBallNow_Y);	// 若机器人目标点横坐标超出给定范围则目标点y不变，目标点x变成keepY
		}
		else {
			Move_Go(footBallNow_X, footBallNow_Y);	// 否则正常运行
		}
	}
	if (getPos().x > keepY2)
	{
		Move_Go(keepY2, footBallNow_Y);	// 若机器人横坐标超出给定范围则y不变，x退回到keepY
	}
	else
	{
		if (footBallNow_X > keepY2)
		{
			Move_Go(keepY2, footBallNow_Y);	// 若机器人目标点横坐标超出给定范围则目标点y不变，目标点x变成keepY

		}
		else
		{
			Move_Go(footBallNow_X, footBallNow_Y);	// 否则正常运行
		}
	}
}

void BaseRobot::PredictRobotInformation(int tick_delay)	// 预测自己n拍后的状态
{
	double delta_t = 1.0;	// 两次调用间隔一拍

	PredictInformation[0] = *robot;	// 预测0拍后的信息应为本拍信息
	AngularSpeed[0] = AngularSpeed[1];	//当前的预测角速度应为对于上一拍而言的下一拍角速度
	for (int i = 1; i <= tick_delay; i++)	// 预测第i拍数据
	{
		double PredictedlastRotation = GetRobotInformation(i - 1).rotation;	//i-1拍旋转角
		Simuro::Vector2 PredictedLastPos = GetRobotInformation(i - 1).position;	// i-1拍位置
		double PredictedLineSpeed = (PredictedLastPos - GetRobotInformation(i - 2).position) / delta_t;// i-1拍线速度
		double angle_logic = atan((PredictedLastPos.y - GetRobotInformation(i - 2).position.y) / (PredictedLastPos.x - GetRobotInformation(i - 2).position.x)) * 180 / M_PI;	//逻辑角度
		if (PredictedLastPos.x < GetRobotInformation(i - 2).position.x)
		{
			if (angle_logic > 0)
			{
				angle_logic = 180 - angle_logic;
			}
			else
			{
				angle_logic = -180 - angle_logic;
			}

		}
		double delta_angle = angle_logic - GetRobotInformation(i - 1).rotation;
		if (delta_angle > 180) delta_angle -= 360;
		if (delta_angle < -180) delta_angle += 360;
		if (fabs(delta_angle) < 90)
		{
			PredictedLineSpeed = -PredictedLineSpeed;	// 根据逻辑角度算出带方向的速度
		}

		double PiedictedAngularSpeed;	// 计算推测的角速度
		if (AngularSpeed[i - 1] > 0)
		{
			if (PredictedlastRotation > GetRobotInformation(i - 2).rotation)
			{
				PiedictedAngularSpeed = (PredictedlastRotation - GetRobotInformation(i - 2).rotation) / delta_t;	//i-1 拍角速度
			}
			else
			{
				PiedictedAngularSpeed = (360 + PredictedlastRotation - GetRobotInformation(i - 2).rotation) / delta_t;
			}
		}
		else
		{
			if (PredictedlastRotation > GetRobotInformation(i - 2).rotation)
			{
				PiedictedAngularSpeed = (PredictedlastRotation - GetRobotInformation(i - 2).rotation - 360) / delta_t;	//i-1 拍角速度
			}
			else
			{
				PiedictedAngularSpeed = (PredictedlastRotation - GetRobotInformation(i - 2).rotation) / delta_t;
			}
		}


		Simuro::Wheel settedWheelSpeed = HistoryInformation[tick_delay + 1 - i].wheel;	// 延迟到第i拍会获得的轮速
		//左右轮速限幅
		if (settedWheelSpeed.leftSpeed > 125)
		{
			settedWheelSpeed.leftSpeed = 125;
		}
		else if (settedWheelSpeed.leftSpeed < -125)
		{
			settedWheelSpeed.leftSpeed = -125;
		}
		if (settedWheelSpeed.rightSpeed > 125)
		{
			settedWheelSpeed.rightSpeed = 125;
		}
		else if (settedWheelSpeed.rightSpeed < -125)
		{
			settedWheelSpeed.rightSpeed = -125;
		}
		//

		// 预测模型的一些参数
		double K1 = 0.002362192;
		double K2 = exp(-1 / 0.9231);
		double K3 = 1 - exp(-1 / 3.096);
		double K4 = 0.53461992;
		double nextLineSpeed;	// 在第i-1拍预测到的i拍线速度

		if (settedWheelSpeed.leftSpeed != 0 && settedWheelSpeed.rightSpeed != 0)
		{
			nextLineSpeed = PredictedLineSpeed * 0.939534127623834133 + (settedWheelSpeed.leftSpeed + settedWheelSpeed.rightSpeed) / 2 * K1;
		}
		if (settedWheelSpeed.leftSpeed == 0 || settedWheelSpeed.rightSpeed == 0)
		{
			nextLineSpeed = PredictedLineSpeed * K2;
		}
		double nextAngularSpeed = PiedictedAngularSpeed + ((settedWheelSpeed.rightSpeed - settedWheelSpeed.leftSpeed) / 2 * K4 - PiedictedAngularSpeed) * K3;
		AngularSpeed[i] = nextAngularSpeed;

		double newLineSpeed = nextLineSpeed;//先算线速度然后根据线速度算出位置？
		double newAngularSpeed = nextAngularSpeed;// 先算角速度然后根据角速度算出角度？（假设角速度ni时针为正）

		PredictInformation[i].position.x = PredictInformation[i - 1].position.x + newLineSpeed * cos(PredictInformation[i - 1].rotation / 180 * M_PI);
		PredictInformation[i].position.y = PredictInformation[i - 1].position.y + newLineSpeed * sin(PredictInformation[i - 1].rotation / 180 * M_PI);
		PredictInformation[i].rotation = PredictInformation[i - 1].rotation + newAngularSpeed;
		while (PredictInformation[i].rotation > 180)
			PredictInformation[i].rotation -= 360;
		while (PredictInformation[i].rotation < -180)
			PredictInformation[i].rotation += 360;

		// 其中历史信息需要单独维护，在main getinstruction每次函数结束前更新历史信息
		// 如果进入摆位函数，则历史信息清零。（reset = true）

	}
}

// 计算在125轮速下Ntick内的累计位移总量
double BaseRobot::calculate_nextNtick_displace_125(double speed, int tickNum)
{
	double total_displace = 0;
	if (tickNum == 0)
	{
		return total_displace;
	}
	else
	{
		double	a = 0.06237304217896197;
		double	b = 0.3045783212622852;
		double v0 = speed;
		total_displace = (b / a) * (1 - exp(-a * (1 + ((log(b) - log(b - v0 * a)) / a))));
		return total_displace + calculate_nextNtick_displace_125(total_displace, tickNum - 1);
	}
}

void BaseRobot::calculate_tangen(double tarx_ball, double tary_ball, double posx, double posy, double ballx, double bally)
{
	// 4tick后球位置
	double ball_future4_x = 4 * ballx - 3 * getLastBallPos().x;
	double ball_future4_y = 4 * bally - 3 * getLastBallPos().y;
	// 目标与球
	double dx = tarx_ball - ball_future4_x;
	double dy = tary_ball - ball_future4_y;
	// 确认击球轨道出轨系数
	double x2 = 20 / sqrt((pow(dx, 2) + pow(dy, 2)));
	// 确认击球轨道出轨点
	double out_circle_x = ball_future4_x - dx * x2;
	double out_circle_y = ball_future4_y - dy * x2;
	// 确认击球轨道圆心系数
	x2 = 24.5 / sqrt((pow(dx, 2) + (pow(dy, 2))));
	int lcircle[] = { 0,0 };
	if (dx > 0) {
		if (dy > 0)
		{
			lcircle[0] = -1;
			lcircle[1] = 1;
		}
		else if (dy < 0)
		{
			lcircle[0] = 1;
			lcircle[1] = 1;
		}
		else if (dy == 0)
		{
			lcircle[0] = 0;
			lcircle[1] = 1;
		}
	}
	else if (dx < 0)
	{
		if (dy > 0)
		{
			lcircle[0] = -1;
			lcircle[1] = -1;
		}
		else if (dy < 0)
		{
			lcircle[0] = 1;
			lcircle[1] = -1;
		}
		else if (dy == 0)
		{
			lcircle[0] = 0;
			lcircle[1] = -1;
		}
	}
	else if (dx == 0)
	{
		if (dy > 0)
		{
			lcircle[0] = -1;
			lcircle[1] = 0;
		}
		else if (dy < 0)
		{
			lcircle[0] = 1;
			lcircle[1] = 0;
		}
		else if (dy == 0)
		{
			lcircle[0] = 0;
			lcircle[1] = 0;
		}
	}
	// 确定左击球轨道圆心点
	// 左圆心
	double lcircle_center_x = out_circle_x + lcircle[0] * fabs(dy) * x2;
	double lcircle_center_y = out_circle_y + lcircle[1] * fabs(dx) * x2;
	// 右圆心
	double rcircle_center_x = out_circle_x - lcircle[0] * fabs(dy) * x2;
	double rcircle_center_y = out_circle_y - lcircle[1] * fabs(dx) * x2;
	// 确认使用的轨道
	double d_pos_lcircle = sqrt(pow(posx - lcircle_center_x, 2) + pow(posy - lcircle_center_y, 2));
	double d_pos_rcircle = sqrt(pow(posx - rcircle_center_x, 2) + pow(posy - rcircle_center_y, 2));
	double circle_center_x = d_pos_lcircle < d_pos_rcircle ? lcircle_center_x : rcircle_center_x;
	double circle_center_y = d_pos_lcircle < d_pos_rcircle ? lcircle_center_y : rcircle_center_y;
	// 确认入轨切点
	// 计算切线长度
	double tangentLen = pow(circle_center_x - posx, 2) + pow(circle_center_y - posy, 2) > pow(24.5, 2) ? sqrt(pow(circle_center_x - posx, 2) + (pow(circle_center_y - posy, 2) - pow(24.5, 2))) : 0;
	// 机器人位置与圆心和x轴的夹角
	double angle_circle_pos = atan2(posy - circle_center_x, posx - circle_center_x) * 180 / M_PI;
	// 机器人位置与圆心和切点的夹角
	double angle_pos_tangent = atan2(tangentLen, 24.5) * 180 / M_PI;
	// 切点坐标
	double angle_final = d_pos_lcircle < d_pos_rcircle ? angle_circle_pos + angle_pos_tangent : angle_circle_pos - angle_pos_tangent;
	double tangentX = circle_center_x + cos((angle_final)*M_PI / 180) * 24.5;
	double tangentY = circle_center_y + sin((angle_final)*M_PI / 180) * 24.5;
	// 机器人与球的距离
	if (fabs(tangentX) > 110 || fabs(tangentY) > 90)
	{
		tangentX = ballx;
		tangentY = bally;
	}
	BaseRobot::tangentX = tangentX;
	BaseRobot::tangentY = tangentY;
	BaseRobot::circle_center_x = circle_center_x;
	BaseRobot::circle_center_y = circle_center_y;
	BaseRobot::ball_future4_x = ball_future4_x;
	BaseRobot::ball_future4_y = ball_future4_y;
	BaseRobot::out_circle_y = out_circle_y;
	BaseRobot::out_circle_x = out_circle_x;
	BaseRobot::d_pos_lcircle = d_pos_lcircle;
	BaseRobot::d_pos_rcircle = d_pos_rcircle;
}



void BaseRobot::shoot_with_angle(double tarx_ball, double tary_ball, double posx, double posy, double ballx, double bally)
{
	calculate_tangen(tarx_ball, tary_ball, posx, posy, ballx, bally);

	// 轨迹逻辑部分
	// 机器人位置与轨道圆心的距离
	double d_pos_cirCter_x = posx - circle_center_x;
	double d_pos_cirCter_y = posy - circle_center_y;
	double d_pos_cirCter = sqrt(pow(d_pos_cirCter_x, 2) + pow(d_pos_cirCter_y, 2));
	// 机器人与切点的距离
	double	d_pos_tan_x = posx - tangentX;
	double	d_pos_tan_y = posy - tangentY;
	double	d_pos_tan = sqrt(pow(d_pos_tan_x, 2) + pow(d_pos_tan_y, 2));
	// 机器人与球的距离
	double	d_pos_ball_x = posx - ball_future4_x;
	double	d_pos_ball_y = posy - ball_future4_y;
	double	d_pos_ball = sqrt(pow(d_pos_ball_x, 2) + pow(d_pos_ball_y, 2));
	// 基于预测的机器人速度
	double	pdx = HistoryInformation[tick_delay].position.x - getLastPos_P().x;
	double	pdy = PredictInformation[tick_delay].position.y - getLastPos_P().y;
	double	pds = sqrt(pow(pdx, 2) + pow(pdy, 2)) < 4.88317245 ? sqrt(pow(pdx, 2) + pow(pdy, 2)) : 4.88317245;
	// 击球方向角与机器人面向角
	double angle_hitball = atan2(ball_future4_y - out_circle_y, ball_future4_x - out_circle_x) * 180 / M_PI;
	double	angle_robot = getLeftWheelVelocity() > 0 ? PredictInformation[tick_delay].rotation : PredictInformation[tick_delay].rotation - 180;
	double	angle_last_robot = getLeftWheelVelocity() > 0 ? getLastRotation() : -getLastRotation();
	if (angle_robot < -180)
	{
		angle_robot += 360;
	}
	if (angle_last_robot < -180)
	{
		angle_last_robot += 360;
	}
	// 跑位控制逻辑
	double	total_displace4 = calculate_nextNtick_displace_125(pds, 4);

	if (d_pos_tan < total_displace4 || d_pos_cirCter < 27.5 || d_pos_ball < 32.5)
	{
		if (((fabs(angle_hitball - angle_robot) < 25) && ((angle_hitball - angle_robot) * (angle_hitball - angle_last_robot)) < 0) || d_pos_ball < 32.5)
		{
			moveTo(tarx_ball, tary_ball);
		}
		else
		{
			if (d_pos_lcircle < d_pos_rcircle)
			{
				if (getLeftWheelVelocity() > 0)
				{
					Velocity(93.5, 125);
				}
				else
				{
					Velocity(-125, -93.5);
				}
			}
			else
			{
				if (getLeftWheelVelocity() > 0)
				{
					Velocity(125, 93.5);
				}
				else
				{
					Velocity(-93.5, -125);
				}
			}
		}
	}
	else
	{
		double	need_tick = 0;
		double	total_dis = 0;
		double	dis_ballx_tick1 = ballx - getLastBallPos().x;
		double	dis_bally_tick1 = bally - getLastBallPos().y;

		int loop = 0;
		while ((d_pos_tan - total_dis) > total_displace4 && loop < 200)
		{
			loop += 1;
			need_tick += 1;
			double	ball_willbe_x = (need_tick + 2) * dis_ballx_tick1 + ballx;
			double	ball_willbe_y = (need_tick + 2) * dis_bally_tick1 + bally;
			calculate_tangen(tarx_ball, tary_ball, posx, posy, ball_willbe_x, ball_willbe_y);
			// 基于预测的机器人速度
			pdx = PredictInformation[tick_delay].position.x - getLastPos_P().x;
			pdy = PredictInformation[tick_delay].position.y - getLastPos_P().y;
			pds = sqrt(pow(pdx, 2) + pow(pdy, 2)) < 4.88317245 ? sqrt(pow(pdx, 2) + pow(pdy, 2)) : 4.88317245;

			d_pos_tan_x = posx - tangentX;
			d_pos_tan_y = posy - tangentY;
			d_pos_tan = sqrt(pow(d_pos_tan_x, 2) + pow(d_pos_tan_y, 2));
			total_dis = calculate_nextNtick_displace_125(pds, need_tick);
		}
		double ball_willbe_x = need_tick * dis_ballx_tick1 + ballx;
		double ball_willbe_y = need_tick * dis_bally_tick1 + bally;

		calculate_tangen(tarx_ball, tary_ball, posx, posy, ball_willbe_x, ball_willbe_y);

		moveTo(tangentX, tangentY);
	}
}

// 保存信息，给下一拍用
void BaseRobot::saveLastInformation(double footBallNow_X, double footBallNow_Y)
{
	lastRotation = robot->rotation;
	if (tick % 10 == 0)
	{
		lastRobotX = robot->position.x;
		lastRobotY = robot->position.y;
	}

}


// 读取某一时刻事件状态
int DataLoader::get_event(int tick)
{
    return this->event_states[tick];
}


// 保存事件及发生时间
void DataLoader::set_tick_state(int tick, int event_state)
{
    this->tick=tick;
    this->event_states[tick]=event_state;
}


