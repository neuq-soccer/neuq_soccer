// Strategy.cpp : 定义 DLL 应用程序的导出函数。
/* 这个文件是主要开发文件，涵盖了策略全部的四个接口
* void GetTeamInfo(TeamInfo* teaminfo)
  用于指定策略信息，目前包含队名字段。
  参数TeamInfo *teaminfo需要策略填充自身的信息,会返回给平台。

- void GetInstruction(Field* field)
  比赛中的每拍被调用，需要策略指定轮速，相当于旧接口的Strategy。
  参数Field* field为In/Out参数，存储当前赛场信息，并允许策略修改己方轮速。
  所有策略的开发应该在此模块

- void GetPlacement(Field* field)
  每次自动摆位时被调用，需要策略指定摆位信息。
  参数Field* field为In/Out参数，存储当前赛场信息，并允许策略修改己方位置（和球的位置）。

- void OnEvent(EventType type, void* argument)
  事件发生时被调用。
  参数EventType type表示事件类型；
  参数void* argument表示该事件的参数，如果不含参数，则为NULL。
*/
#include "../header/stdafx.h"
#include "../header/platform.h"
#include "../header/adapter.h"
#include "../header/BaseRobot.h"
#include "../header/globalVariable.h"
#include<iostream>
#include "xstring"
#include<string>
#include<typeinfo>
#include<sstream>
#include<locale>
#include<vector>
using namespace Simuro;
using namespace Adapter;
using namespace std;

int tick = 0;
short tick_delay = 4;
extern int tickBeginPenalty = 0;
extern int tickBeginGoalKick = 0;
BaseRobot baseRobots[5];	// 我方机器人数组
BaseRobot oppRobots[5];	//对方机器人数组
DataLoader dataloader;
int race_state = -1;//处于何种定位球状态，0是开球，其他遵从JudgeType
int race_state_trigger = -1;//哪一方触发了定位球
void ConvertFieldToOtherSide(Field* field);
double* cal_robot_dis(BaseRobot *robot1, double ballx, double bally);
Vector2 BallPos[100000] = { {0,0} };
bool resetHistoryRecord = false;
bool newMatch = false;

/**
* 打印比赛状态
*/
void OnEvent(EventType type, void* argument) {
	SendLog(L"V/Strategy:OnEvent()");
	if (type == EventType::JudgeResult)
	{
		JudgeResultEvent* judgeResult = static_cast<JudgeResultEvent*>(argument);
		race_state = judgeResult->type;
		race_state_trigger = judgeResult->actor;
		if (judgeResult->type == JudgeType::PlaceKick) 	// 判断是否进入开球
		{
			SendLog(L"Place Kick");
		}
		switch (judgeResult->actor) {	// 判断是否是进攻方
		case Team::Self:
			SendLog(L"By self");
			break;
		case Team::Opponent:
			SendLog(L"By opp");
			break;
		case Team::None:
			SendLog(L"By both");
			break;
		}
	}
}

/**
* 获得队伍信息
*/
void GetTeamInfo(TeamInfo* teamInfo) {
	SendLog(L"V/Strategy:GetTeamInfo()");
	static const wchar_t teamName[] = L"1v1-demo-2023";	// 在此行修改双引号中的字符串为自己的队伍名
	static constexpr size_t len = sizeof(teamName);
	memcpy(teamInfo->teamName, teamName, len);
}

/**
* 摆位信息，进行定位球摆位
*/
void GetPlacement(Field* field) {
	ConvertFieldToOtherSide(field);
	SendLog(L"V/Strategy:GetPlacement()");
	auto robots = field->selfRobots;

	if (race_state == JudgeType::PlaceKick)//开球
	{
		SendLog(L"开球摆位");
		if (race_state_trigger == Team::Self)//开球进攻
		{
			robots[0].position.x = 0;
			robots[0].position.y = 15;
			robots[0].rotation = -90;
			robots[1].position.x = 42;
			robots[1].position.y = 42;
			robots[1].rotation = 180;
			robots[2].position.x = -30;
			robots[2].position.y = -10;
			robots[2].rotation = 0;
			robots[3].position.x = -50;
			robots[3].position.y = 10;
			robots[3].rotation = 0;
			robots[4].position.x = -80;
			robots[4].position.y = 0;
			robots[4].rotation = 0;
		}
		else if (race_state_trigger == Team::Opponent)//开球防守
		{
			robots[0].position.x = -30;
			robots[0].position.y = 0;
			robots[0].rotation = 0;
			robots[1].position.x = -10;
			robots[1].position.y = 80;
			robots[1].rotation = -90;
			robots[2].position.x = -30;
			robots[2].position.y = -80;
			robots[2].rotation = -90;
			robots[3].position.x = -50;
			robots[3].position.y = 70;
			robots[3].rotation = -90;
			robots[4].position.x = -80;
			robots[4].position.y = -80;
			robots[4].rotation = -90;
		}
		else//None人触发
		{

		}
	}
	if (race_state == PenaltyKick)	// 点球
	{
		if (race_state_trigger == Team::Self)	//点球进攻
		{
			robots[0].position.x = -103;
			robots[0].position.y = 0;
			robots[0].rotation = 90;
			robots[1].position.x = 30;
			robots[1].position.y = 0;
			robots[1].rotation = 0;
			robots[2].position.x = -3;
			robots[2].position.y = -10;
			robots[2].rotation = 0;
			robots[3].position.x = -3;
			robots[3].position.y = 10;
			robots[3].rotation = 0;
			robots[4].position.x = -3;
			robots[4].position.y = 0;
			robots[4].rotation = 0;
		}
		if (race_state_trigger == Team::Opponent)	// 点球防守
		{
			robots[0].position.x = -105;
			robots[0].position.y = 0;
			robots[0].rotation = 0;
			robots[1].position.x = 30;
			robots[1].position.y = 0;
			robots[1].rotation = 0;
			robots[2].position.x = 10;
			robots[2].position.y = -10;
			robots[2].rotation = 0;
			robots[3].position.x = 10;
			robots[3].position.y = 10;
			robots[3].rotation = 0;
			robots[4].position.x = 10;
			robots[4].position.y = 0;
			robots[4].rotation = 0;
		}
	}
	if (race_state == GoalKick)	// 门球
	{
		if (race_state_trigger == Team::Self)	//门球进攻
		{
			robots[0].position.x = -103;
			robots[0].position.y = 0;
			robots[0].rotation = 0;
			robots[1].position.x = 30;
			robots[1].position.y = 0;
			robots[1].rotation = 0;
			robots[2].position.x = -30;
			robots[2].position.y = -10;
			robots[2].rotation = 0;
			robots[3].position.x = -50;
			robots[3].position.y = 10;
			robots[3].rotation = 0;
			robots[4].position.x = -80;
			robots[4].position.y = 0;
			robots[4].rotation = 0;
			field->ball.position.x = -110 + 15;
			field->ball.position.y = 0;
		}
		if (race_state_trigger == Team::Opponent)	// 门球防守
		{
			robots[0].position.x = -105;
			robots[0].position.y = 0;
			robots[0].rotation = 0;
			robots[1].position.x = 30;
			robots[1].position.y = 0;
			robots[1].rotation = 0;
			robots[2].position.x = -30;
			robots[2].position.y = -10;
			robots[2].rotation = 0;
			robots[3].position.x = -50;
			robots[3].position.y = 10;
			robots[3].rotation = 0;
			robots[4].position.x = -80;
			robots[4].position.y = 0;
			robots[4].rotation = 0;
			field->ball.position.x = 0;
			field->ball.position.y = 0;
		}
	}
	if (race_state == FreeKickLeftBot || race_state == FreeKickLeftTop
		|| race_state == FreeKickRightBot || race_state == FreeKickRightTop)	// 争球
	{
		if (race_state_trigger == Team::Self)	// 争球进攻
		{
			robots[0].position.x = -3;
			robots[0].position.y = -10;
			robots[0].rotation = 0;
			robots[1].position.x = 30;
			robots[1].position.y = 0;
			robots[1].rotation = 0;
			robots[2].position.x = -3;
			robots[2].position.y = -10;
			robots[2].rotation = 0;
			robots[3].position.x = -3;
			robots[3].position.y = 10;
			robots[3].rotation = 0;
			robots[4].position.x = -3;
			robots[4].position.y = 0;
			robots[4].rotation = 0;
			field->ball.position.x = 0;
			field->ball.position.y = 0;
		}
		if (race_state_trigger == Team::Opponent)	// 争球防守
		{
			robots[0].position.x = 10;
			robots[0].position.y = 10;
			robots[0].rotation = 0;
			robots[1].position.x = 30;
			robots[1].position.y = 0;
			robots[1].rotation = 0;
			robots[2].position.x = 10;
			robots[2].position.y = -10;
			robots[2].rotation = 0;
			robots[3].position.x = 10;
			robots[3].position.y = 10;
			robots[3].rotation = 0;
			robots[4].position.x = 10;
			robots[4].position.y = 0;
			robots[4].rotation = 0;
			field->ball.position.x = 0;
			field->ball.position.y = 0;
		}
	}

	ConvertFieldToOtherSide(field);
}


// 策略行为主函数，可将以下函数用策略模式封装
void strategy(Field* field)
{
	double footBallNow_X = field->ball.position.x;
	double footBallNow_Y = field->ball.position.y;


	// 预测足球位置
	double futureBallx = 4 * footBallNow_X - 3 * BallPos[tick - 1].x;
	double futureBally = 4 * footBallNow_Y - 3 * BallPos[tick - 1].y;
	if (tick == 1)
	{
		baseRobots[0].Velocity(0, 0);
	}

	double* my_dis = cal_robot_dis(&baseRobots[0], futureBallx, futureBally);
	double* opp_dis = cal_robot_dis(&oppRobots[0], futureBallx, futureBally);
	if (my_dis[2] < opp_dis[2])
	{
		baseRobots[0].shoot_with_angle(-110, 0, baseRobots[0].PredictInformation[tick_delay].position.x, baseRobots[0].PredictInformation[tick_delay].position.y, footBallNow_X, footBallNow_Y);
		//baseRobots[1].shoot_with_angle(-110, 0, baseRobots[0].PredictInformation[tick_delay].position.x, baseRobots[0].PredictInformation[tick_delay].position.y, footBallNow_X, footBallNow_Y);
		//baseRobots[2].shoot_with_angle(-110, 0, baseRobots[0].PredictInformation[tick_delay].position.x, baseRobots[0].PredictInformation[tick_delay].position.y, footBallNow_X, footBallNow_Y);
		//baseRobots[3].shoot_with_angle(-110, 0, baseRobots[0].PredictInformation[tick_delay].position.x, baseRobots[0].PredictInformation[tick_delay].position.y, footBallNow_X, footBallNow_Y);
		//baseRobots[4].shoot_with_angle(-110, 0, baseRobots[0].PredictInformation[tick_delay].position.x, baseRobots[0].PredictInformation[tick_delay].position.y, footBallNow_X, footBallNow_Y);
	}
	else
	{
		baseRobots[0].shoot(futureBallx, futureBally);
		//baseRobots[1].shoot(futureBallx, futureBally);
		//baseRobots[2].shoot(futureBallx, futureBally);
		//baseRobots[3].shoot(futureBallx, futureBally);
		//baseRobots[4].shoot(futureBallx, futureBally);
	}
	delete[]my_dis;
	delete[]opp_dis;

	//baseRobots[0].Move_Go(footBallNow_X, footBallNow_Y);
	//baseRobots[0].Move_Go(20, 20);
	//baseRobots[0].moveto_within_x_limits(80, footBallNow_X, footBallNow_Y);
	//存储信息
	for (int i = 0; i < 5; i++)
	{
		baseRobots[i].saveLastInformation(footBallNow_X, footBallNow_Y);
	}
}

// 球距离计算函数(基于预测)
double* cal_robot_dis(BaseRobot *robot1, double ballx, double bally)
{
	double *ds = new double[3];
	double	dx = robot1->PredictInformation[tick_delay].position.x - ballx;
	double	dy = robot1->PredictInformation[tick_delay].position.y - bally;
	ds[0] = dx;
	ds[1] = dy;
	ds[2] = sqrt(pow(dx, 2) + pow(dy, 2));
	return ds;
}

/**
* 获得策略信息
* 策略接口，相当于策略执行的主模块，可以不恰当地理解为main函数，是主要开发的部分
*/
void GetInstruction(Field* field) {
	ConvertFieldToOtherSide(field);
	tick = field->tick;

	for (int i = 0; i < 5; i++) {
		baseRobots[i].update(&(field->selfRobots[i]));	// 每一拍更新己方机器人信息给BaseRobot
	}
	for (int i = 0; i < 5; i++)
	{
		oppRobots[i].update(&(field->opponentRobots[i]));	// 每一拍更新敌方机器人信息给OppRobot
	}

	double footBallNow_X = field->ball.position.x;
	double footBallNow_Y = field->ball.position.y;
	BallPos[tick] = { float(footBallNow_X), float(footBallNow_Y) };	// 记录球的位置

	strategy(field);	// 执行策略
	//bool test_method = 0;

	//baseRobots[0].moveTo(footBallNow_X, footBallNow_Y);
	dataloader.set_tick_state(tick, race_state);
}

/**
 * @brief 将作为传入的数据转换为另一方视角
 *
 * @param field 赛场数据
 */
void ConvertFieldToOtherSide(Field* field) {
	//field->ball.position.x *= -1;
	//field->ball.position.y *= -1;
	//for (int i = 0; i < PLAYERS_PER_SIDE; i++)
	//{

	//	field->opponentRobots[i].position.x *= -1;
	//	field->opponentRobots[i].position.y *= -1;
	//	field->opponentRobots[i].rotation = field->opponentRobots[i].rotation > 0 ? -180 + field->opponentRobots[i].rotation : 180 + field->opponentRobots[i].rotation;
	//	field->selfRobots[i].position.x *= -1;
	//	field->selfRobots[i].position.y *= -1;
	//	field->selfRobots[i].rotation = field->selfRobots[i].rotation > 0 ? -180 + field->selfRobots[i].rotation : 180 + field->selfRobots[i].rotation;
	//}
}
