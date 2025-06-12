#pragma once
#include <cstdint>
// 平台信息
namespace Simuro {

	static const int PLAYERS_PER_SIDE = 5;
	static const int MAX_STRING_LEN = 128;
	// 定义一些事件类型：
	// 判罚结果，比赛开始，比赛暂停，上半场开始，下半场开始，加时开始，5v5比赛的点球大战开始，点球大战比赛开始，突破重围开始

	enum EventType {
		JudgeResult = 0,
		MatchStart = 1,
		MatchStop = 2,
		FirstHalfStart = 3,
		SecondHalfStart = 4,
		OvertimeStart = 5,
		PenaltyShootoutStart = 6,
		MatchShootOutStart = 7,
		MatchBlockStart = 8,
	};

	// 定义一些判罚类型：
	// 开球，门球，点球，争球

	enum JudgeType
	{
		PlaceKick = 0,
		GoalKick = 1,
		PenaltyKick = 2,
		FreeKickRightTop = 3,
		FreeKickRightBot = 4,
		FreeKickLeftTop = 5,
		FreeKickLeftBot = 6,
	};

	// 定义队伍：
	// 我方，对方
	// 一般用于调用各方机器人或判断定位球执行者

	enum Team
	{
		Self,
		Opponent,
		None,
	};
	// 定义坐标的二维向量
	struct Vector2 {
		float x;
		float y;
	};

	// 定义队伍信息，即队名 
	struct TeamInfo {
		wchar_t teamName[MAX_STRING_LEN];
	};

	// 定义球的位置 
	struct Ball {
		Vector2 position;
	};

	// 定义左右轮速 
	struct Wheel {
		float leftSpeed;
		float rightSpeed;
	};

	// 定义机器人属性：
	// 坐标，旋转角，轮（速）

	struct Robot {
		Vector2 position;
		float rotation;
		Wheel wheel;
	};

	// 定义环境：
	// 我方机器人数组，对方机器人数组，球，当前拍数（每秒66拍）

	struct Field {
		Robot selfRobots[PLAYERS_PER_SIDE];
		Robot opponentRobots[PLAYERS_PER_SIDE];
		Ball ball;
		int32_t tick;
	};

	// 定义判罚事件：
	//判罚类型，执行方，判罚原因
	struct JudgeResultEvent {
		JudgeType type;
		Team actor;  //用来区分是对手还是队友的枚举
		wchar_t reason[MAX_STRING_LEN];
	};
}
