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

// 用于获取机器人旋转角
double BaseRobot::getRotation()
{
	return robot->rotation;
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
	x_dis->proportion = 0.30;
	x_dis->integral = 0.001;
	x_dis->derivative = 1.30;
	x_dis->i_Band = 200;
	x_dis->i_max = 500;
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
	pidCal(sptr, robot->rotation, Tar_Angle);
	double speedr, speedl;
	speedr = sptr->total_out;
	speedl = -sptr->total_out;
	Velocity(speedl, speedr);
}

double speedr = 0, speedl = 0, speed_tr = 0, speed_tl = 0;
void BaseRobot::Move_Go(double Tar_x, double Tar_y)
{
	if (tick == 1 || tick == 2 || tick % 100 == 0)
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
	double vel = Tar_distance * Kp;
	pidCal(sptr, robot->rotation, tar_angle);
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
		speedl = -60;
		speedr = 60;
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
		this->moveTo(x_limits, tar_y);	// 跑回限制界里面
	}
	else
	{
		if (tar_x > x_limits)	// 如果跑位目标点超过x_limits
		{
			this->moveTo(x_limits, tar_y);	// 改变跑位目标点，往限制界里面跑
		}
		else
		{
			this->moveTo(tar_x, tar_y);	// 否则正常运行
		}
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


