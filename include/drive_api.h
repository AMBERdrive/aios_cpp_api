#ifndef DRIVE_API_H
#define DRIVE_API_H

#include <memory>
#include <string.h>
#include <vector>
#include <iostream>
#include <jsoncpp/json/json.h> 
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

using namespace std;



/// \brief AMBER API 
///
///
namespace Amber{

class TrapzoidalTrajectoryParameters
{
	Eigen::VectorXd acc;
	Eigen::VectorXd dec;
	Eigen::VectorXd vel;
};

enum ControlMode
{
	kCurrentMode = 1,/**< 电流控制模式 */
	kVelocityMode = 2,/**< 速度控制模式 */
	kPositionMode = 3,/**< 位置控制模式 */
};

class AiosAttribute
{
public:
	std::string ip_; /**< ip地址 */ 
	std::string mac_address_;/**< mac地址 */
	std::string serial_number_;/**< 执行器序列号 */
	std::string fw_version_;/**< 固件版本号 */
	std::string hw_version_;/**< 硬件版本号 */
	int m_;///< m0=0/m1=非0 */
	int id_;
	std::string name_;
	bool drive_status_;
	AiosAttribute& operator = (const AiosAttribute& attribute);
};

class CvpFeedbackData
{
public:
	Eigen::VectorXd pos;
	Eigen::VectorXd vel;
	Eigen::VectorXd current;
};

class AiosGroup final
{
private:
	int axis_num_;
	vector <AiosAttribute> attribute_;
	vector<string> ip_list_;
	vector<string> serial_number_list_;
	vector<string> mac_address_list_;
	vector<string> name_list_;

	vector<int> id_list_;
	vector<int> m_list_;
	vector<bool> drive_status_;

	int periodic_time_;

	void CommunicationOnce(const Json::Value send_data,Json::Value &recv_data,int axis,int port=2334);
	void SendTo(const Json::Value send_data[],int port=2334);
	void SendTo(vector <int> index,const Json::Value send_data[],int port);

	void RecvFrom(Json::Value recv_data[] ,int port=2334);
	void RecvFrom(vector <int> index,Json::Value recv_data[] ,int port);
	void RecvFromNoneM(Json::Value recv_data[] ,int port);

	void ClearSocketBuffer();
	int IsEncoderReady(bool &flag,vector <int> index);
	int GetMotionControllerConfig(Eigen::VectorXd &kp,vector <int> index,int mode);
	int SetMotionControllerConfig(Eigen::VectorXd kp , vector <int> index,int mode);
	int GetMotorConfig(Eigen::VectorXd &kp , vector <int> index,int mode);	
	int SetMotorConfig(Eigen::VectorXd kp , vector <int> index,int mode);	

public:
	
	AiosGroup();
	~AiosGroup();
	void RequsestCvpFeedback();

	int ResponseCvpRequest(	 CvpFeedbackData &fb);

	void Initialize(const vector <AiosAttribute> attribute);
	int DisableVelocityRampMode();
	int SetRampedVelocity(const Eigen::VectorXd vel,CvpFeedbackData &fb);
	int EnableVelocityRampMode();
	int SetCurrentRequest(const Eigen::VectorXd current);
	int SetIo(int value);
	int SetRampedVelocity(const Eigen::VectorXd vel);
	int GetControlMode();

	/**
	 * @brief 获得轴组内执行器的个数
	 * @return 轴组内执行器的个数
	 */
	int Size()const;
	
	/**
	 * @brief 获取轴组内执行器的具体信息
	 * @return 轴组内执行器的具体信息
	 */
	vector <AiosAttribute> GetActuatorInfo();

	/**
	 * @brief 获取当前位置
	 * @param[out] pos 当前位置(单位:count)
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int GetPosition(Eigen::VectorXd &pos);

	/**
	 * @brief 获取当前速度
	 * @param[out] vel 当前速度(单位:count/s)
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int GetVelocity(Eigen::VectorXd &vel);

	/**
	 * @brief 获取当前电流
	 * @param[out] ccurrent 当前电流(单位:A)
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int GetCurrent(Eigen::VectorXd &current);

	/**
	 * @brief 获取当前位置、速度和电流
	 *
	 * @param[out] pos 当前位置(单位:count)
	 * @param[out] vel 当前速度(单位:count/s)
	 * @param[out] current 当前电流(单位:A)
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int GetCvp(	 CvpFeedbackData &fb);

	/**
	 * @brief 是否使能
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int IsEnable(vector <bool> &is_enabled);

	/**
	 * @brief 使能
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int Enable();

	/**
	 * @brief 失能
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int Disable();
	
	/**
	 * @brief 把当前点设置为零点
	 * @attention 请尽量在失能状态下设置零位
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int SetHomePosition();

	/**
	 * @brief 使轴组运动到目标位置
	 * @param[in] pos 目标位置(单位:count)
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int SetPosition(const Eigen::VectorXd pos);

	/**
	 * @brief 使轴组运动到目标位置并返回当前位置、速度、电流
	 * @param[in] pos 目标位置(单位:count)
	 * @param[out] fb 当前位置、电流和速度
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int SetPosition(const Eigen::VectorXd pos,CvpFeedbackData &fb);

	/**
	 * @brief 使执行器达到目标速度
	 * @param[in] vel 目标速度(单位:count/s)
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int SetVelocity(const Eigen::VectorXd vel);

	/**
	 * @deprecated
	 * @brief 使执行器达到目标速度
	 * @param[in] vel 目标速度(单位:count/s)
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int SetVelocity(const Eigen::VectorXd vel,CvpFeedbackData &fb);

	/**
	 * @brief 使执行器达到目标电流
	 * @param[in] current 目标电流(单位:A)
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int SetCurrent(const Eigen::VectorXd current);

	/**
	 * @brief 使执行器达到目标电流
	 * @param[in] current 目标电流(单位:A)
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int SetCurrent(const Eigen::VectorXd current,CvpFeedbackData &fb);

	/**
	 * @brief 执行器标定
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int Calibration();

	/**
	 * @brief 梯形加减速运动到指定点
	 * @param[in] pos 目标位置(单位:count)
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int TrapezoidalMove( const Eigen::VectorXd pos);

	/**
	 * @brief 设置梯形加减速参数
	 * @param[in] acc 最大加速度(单位:count/s)
	 * @param[in] dec 最大减速度(单位:count/s)
	 * @param[in] vel 最大速度(单位:count/s)
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int SetTrapzoidalTrajectoryParameters(const Eigen::VectorXd acc,const Eigen::VectorXd dec,const Eigen::VectorXd vel);

	/**
	 * @brief 设置梯形加减速参数
	 * @param[in] acc 最大加速度(单位:count/s)
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int SetTrapzoidalTrajectoryAcc(const Eigen::VectorXd acc);

	/**
	 * @brief 设置梯形加减速参数
	 * @param[in] dec 最大减速度(单位:count/s)
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int SetTrapzoidalTrajectoryDec(const Eigen::VectorXd dec);

	/**
	 * @brief 设置梯形加减速参数
	 * @param[in] vel 最大速度(单位:count/s)
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int SetTrapzoidalTrajectoryVel(const Eigen::VectorXd vel);

	/**
	 * @brief 获取梯形加减速参数
	 * @param[out] acc 最大加速度(单位:count/s)
	 * @param[out] dec 最大减速度(单位:count/s)
	 * @param[out] vel 最大速度(单位:count/s)
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int GetTrapzoidalTrajectoryParameters(Eigen::VectorXd &acc,Eigen::VectorXd &dec,Eigen::VectorXd &vel);

	/**
	 * @brief 获取梯形加减速参数
	 * @param[out] acc 最大加速度(单位:count/s)
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int GetTrapzoidalTrajectoryAcc(Eigen::VectorXd &acc);

	/**
	 * @brief 获取梯形加减速参数
	 * @param[out] dec 最大减速度(单位:count/s)
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int GetTrapzoidalTrajectoryDec(Eigen::VectorXd &dec);

	/**
	 * @brief 获取梯形加减速参数
	 * @param[out] vel 最大速度(单位:count/s)
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int GetTrapzoidalTrajectoryVel(Eigen::VectorXd &vel);

	/**
	 * @brief 设置运动控制模式
	 * @param[in] mode 注意mode应和index维数一致， kCurrentMode:电流环 kVelocityMode:速度环 kPositionMode:位置环
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int SetControlMode(const ControlMode mode);

	/**
	 * @brief 获取指定执行器位置环比例量
	 * @param[out] kp 位置环比例量
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int GetPostionKp(Eigen::VectorXd & kp);

	/**
	 * @brief 获取指定执行器速度环比例量
	 * @param[out] kp 速度环比例量
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int GetVelocityKp(Eigen::VectorXd &kp);

	/**
	 * @brief 获取指定执行器速度环积分量
	 * @param[out] ki 速度环积分量
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int GetVelocityKi(Eigen::VectorXd &ki);

	/**
	 * @brief 获取指定执行器最大速度
	 * @param[out] limit 最大速度
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int GetVelocityLimit(Eigen::VectorXd &limit);

	/**
	 * @brief 设置指定执行器位置环比例量
	 * @param[in] kp 位置环比例量
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int SetPostionKp(const Eigen::VectorXd kp);

	/**
	 * @brief 设置指定执行器速度环比例量
	 * @param[in] kp 速度环比例量
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int SetVelocityKp(const Eigen::VectorXd kp);

	/**
	 * @brief 设置指定执行器速度环积分量
	 * @param[in] ki 速度环积分量
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int SetVelocityKi(const Eigen::VectorXd ki);

	/**
	 * @brief 设置指定执行器最大速度
	 * @param[in] limit 最大速度
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int SetVelocityLimit(const Eigen::VectorXd limit);

	/**
	 * @brief 获取指定执行器最大电流
	 * @param[out] limit 最大电流
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int GetCurrentLimit(Eigen::VectorXd &limit);

	/**
	 * @brief 获取指定执行器最大电流环带宽
	 * @param[out] bandwidth 最大电流环带宽
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int GetCurrentBandwidth(Eigen::VectorXd &bandwidth);

	/**
	 * @brief 设置指定执行器最大电流
	 * @param[in] limit 最大电流
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int SetCurrentLimit(const Eigen::VectorXd limit);

	/**
	 * @brief 设置指定执行器最大电流环带宽
	 * @param[in] bandwidth 最大电流环带宽
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int SetCurrentBandwidth(const Eigen::VectorXd bandwidth);

	/**
	 * @brief 清除修改的配置
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int ClearConfig();

	/**
	 * @brief 保存配置
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int SaveConfig();

	/**
	 * @brief 重启
	 * @attention 重启轴组内执行器，重启完成后需要重连
	 */
	void Reboot();

	/**
	 * @brief 获取报错信息
	 * @return 轴组内各执行器错误信息
	 */
	vector <string> GetErrorDetails();


	/**
	 * @brief 清除轴组内的报错信息
	 * @attention 确定解除错误源头后清除才有效
	 */
	void ClearError();
	
	/**
	 * @brief 固件升级
	 * @return 执行成功与否
	 *	 @retval 0 成功 
	 *	 @retval -1 失败
	 */
	int FirmwareUpdate();

public:	
	int SetPositionRequest(Eigen::VectorXd pos);
	
};


class Motion
{
public:

	/**
	 * @brief 初始化运动停止信号
	 *
	 */
	static void InitStopSignal();	

	/**
	 * @brief 发送运动停止信号
	 *
	 */
	static void SetStopSignal();	

	/**
	 * @brief 获取运动停止信号
	 *
	 */
	static bool GetStopSignal();

	/**
	 * @brief 使轴组步进一段位移
	 * @param group 轴组对象
	 * @param pos 位移 单位（count）
	 * @return 返回说明
	 *	 @retval 0 成功 
	 *	 @retval 非0 错误码 
	 */
	static int MoveStep(AiosGroup * unit, const Eigen::VectorXd pos);

	/**
	 * @brief 使轴组运行到目标点
	 * @param group 轴组对象
	 * @param pos 目标点 单位（count）
	 * @return 返回说明
	 *	 @retval 0 成功 
	 *	 @retval 非0 错误码 
	 */
	static int MoveTo(AiosGroup * group,const Eigen::VectorXd pos);

	/**
	 * @brief 记录示教的轨迹
	 * @param group 轴组对象
	 * @param file_path 轨迹文件
	 * @return 返回说明
	 *	 @retval 0 成功 
	 *	 @retval 非0 错误码 
	 */
	static int RecordPoint(AiosGroup * group, const std::string file_path="data.txt");	

	/**
	 * @brief 再现记录的轨迹
	 * @param group 轴组对象
	 * @param file_path 轨迹文件
	 * @return 返回说明
	 *	 @retval 0 成功 
	 *	 @retval 非0 错误码 
	 */
	static int Replay(AiosGroup * group,const std::string file_path="data.txt",const int count=0);
};

class Lookup
{
private:
	int Broadcast();
	vector <AiosAttribute> attribute_;
public:
	Lookup();
	~Lookup();

	/**
	 * @brief 获取可用的轴组对象
	 * @return 轴组对象
	 *	 @retval NULL 失败 
	 *	 @retval 非NUll 成功 
	 */
	std::shared_ptr <AiosGroup> GetAvailableList();

	/**
	 * @brief 按执行器序列号获取轴组对象
	 * @param[in] serial_number 输入的序列号
	 * @return 轴组对象
	 *	 @retval NULL 失败 
	 *	 @retval 非NUll 成功 
	 */
	std::shared_ptr <AiosGroup> GetHandlesFromSerialNumberList(const std::vector <string> serial_number);

	/**
	 * @brief 按mac地址获取轴组对象
	 * @param[in] mac_address 输入的mac地址
	 * @return 轴组对象
	 *	 @retval NULL 失败 
	 *	 @retval 非NUll 成功 
	 */
	std::shared_ptr <AiosGroup> GetHandlesFromMacAddressList(const std::vector <string> mac_address);
};

/**
 * @brief 获取系统错误
 * @return 错误描述
 */
string GetSystemError();

}

#endif
