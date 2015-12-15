#include "rosRoboteqDrv.h"

typedef std::vector<std::string> TStrVec;
void    Split(TStrVec& vec, const string& str);

RosRoboteqDrv::RosRoboteqDrv(void)
 : _logEnabled(false), _comunicator(*this, *this)
{
}

bool    RosRoboteqDrv::Initialize()
{
    try
    {
        std::string mode;

        if (ros::param::get("~mode", mode) == false )
        {
            ROS_FATAL_STREAM_NAMED(NODE_NAME, " Please specify mode parameter");
            return false;
        }

        std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);

        std::string device;

        if (ros::param::get("~device", device) == false )
        {
            ROS_FATAL_STREAM_NAMED(NODE_NAME, " Please specify device parameter");
            return false;
        }

        if (ros::param::get("~left", _left) == false )
        {
            ROS_FATAL_STREAM_NAMED(NODE_NAME, " Please specify left parameter");
            return false;
        }

        if (ros::param::get("~right", _right) == false )
        {
            ROS_FATAL_STREAM_NAMED(NODE_NAME, " Please specify right parameter");
            return false;
        }

        ROS_INFO_STREAM_NAMED(NODE_NAME, "Channels Right: " << _right << ", Left: " << _left);

        _pub = _nh.advertise<geometry_msgs::Twist>("current_velocity", 1); 

        _service = _nh.advertiseService("set_actuators", &RosRoboteqDrv::SetActuatorPosition, this); 
//        _service = _nh.advertiseService("manual_CAN_command", &RosRoboteqDrv::ManualCANCommand, this);

        // Do not remove below line. Else it will
        // not print diag msg from lower libs
        _logEnabled = true;

        if( mode == "can" )
        	_comunicator.Open(RoboteqCom::eCAN, device);
        else
        	_comunicator.Open(RoboteqCom::eSerial, device);
        
        if(_comunicator.Version().empty() )
            THROW_RUNTIME_ERROR("Failed to receive Roboteq Version");

        if(_comunicator.Model().empty() )
            THROW_RUNTIME_ERROR("Failed to receive Roboteq Model");

        if(_comunicator.IsThreadRunning() == false)
            THROW_RUNTIME_ERROR("Failed to spawn RoboReader Thread");
        
        _comunicator.IssueCommand("# C");   // Clears out telemetry strings

	    if( _comunicator.Mode() == RoboteqCom::eSerial )
	    {
            _comunicator.IssueCommand("?S");    // Query for speed and enters this speed 
                                            // request into telemetry system
            _comunicator.IssueCommand("# 100"); // auto message response is 500ms
	    }

        _sub = _nh.subscribe("cmd_vel", 1, &RosRoboteqDrv::CmdVelCallback, this);
        _buttonSub = _nh.subscribe("xbox_controller", 1, &RosRoboteqDrv::XButtonCallback, this);
    }
    catch(std::exception& ex)
    {
        ROS_ERROR_STREAM_NAMED(NODE_NAME,"Open Port Failed. Error: " << ex.what());
        throw;
    }

    return true;
}

void    RosRoboteqDrv::Shutdown(void)
{
    _comunicator.Close();
}

void    RosRoboteqDrv::XButtonCallback(const base_controller::Xbox_Button_Msg::ConstPtr& buttons)
{
    std::stringstream ss;

    if( _comunicator.Mode() == RoboteqCom::eCAN )
    {
        if(buttons->a != 0)
        {
            ROS_INFO("--Going to DIG position--");
            ss << "@04!G 1 900_@04!G 2 900";
        }
        else if(buttons->y != 0)
        {
            ROS_INFO("--Going to DUMP position--");
            ss << "@04!G 1 -1000_@04!G 2 -1000";
        }
        else if(buttons->b != 0)
        {
            ROS_INFO("--Going to DRIVE position--");
            ss << "@04!G 1 0_@04!G 2 0";
        }
    }

    try
    {
        _comunicator.IssueCommand(ss.str());
        ROS_INFO_STREAM("Actuator= " << ss.str());
    }
    catch(std::exception& ex)
    {
        ROS_ERROR_STREAM_NAMED(NODE_NAME,"IssueCommand : " << ex.what());
        throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM_NAMED(NODE_NAME,"IssueCommand : ?");
        throw;
    }

}

void    RosRoboteqDrv::CmdVelCallback(const geometry_msgs::Twist::ConstPtr& twist_velocity)
{
    _wheelVelocity = ConvertTwistToWheelVelocity(twist_velocity);

    float leftVelRPM  = _wheelVelocity.left  / RPM_TO_RAD_PER_SEC;
    float rightVelRPM = _wheelVelocity.right / RPM_TO_RAD_PER_SEC;

    // now round the wheel velocity to int

    std::stringstream ss;

    if( _comunicator.Mode() == RoboteqCom::eSerial )
    {
        ss << "!G " << _left << " " << (((int)leftVelRPM) * 100);
        ss << "_!G " << _right << " " << (((int)rightVelRPM) * 100);
    }
    else
    {
       // ss << "@00!G " << _left << " " << (((int)leftVelRPM) * 100);
       // ss << "_@00!G " << _right << " " << (((int)rightVelRPM) * 100);
        
        for( int i = 1; i <= 3; i++) // 3 is number of wheel pairs
        {
            if( i != 1 )
                ss << "_";

            ss <<  "@0" << i << "!G " << _left  << " " << (((int)leftVelRPM)  * 100);
            ss << "_@0" << i << "!G " << _right << " " << (((int)rightVelRPM) * 100);
        }
    }

    try
    {
        _comunicator.IssueCommand(ss.str());
        ROS_INFO_STREAM("Wheels= " << ss.str());
    }
    catch(std::exception& ex)
    {
        ROS_ERROR_STREAM_NAMED(NODE_NAME,"IssueCommand : " << ex.what());
	    throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM_NAMED(NODE_NAME,"IssueCommand : ?");
	    throw;
    }
}

bool RosRoboteqDrv::SetActuatorPosition(TSrvAct_Req &req, TSrvAct_Res &res)
{
    std::stringstream ss;

    if( _comunicator.Mode() == RoboteqCom::eCAN )
    {
        ss << "@04!G 1 " << req.actuator_position;
        ss << "_@04!G 2 " << req.actuator_position;

        // ss << "@00!G 1 " << req.actuator_position;
        // ss << "_@00!G 2 " << req.actuator_position;
    }

    try
    {
        _comunicator.IssueCommand(ss.str());
        ROS_INFO_STREAM("Actr= " << ss.str());
    }
    catch(std::exception& ex)
    {
        ROS_ERROR_STREAM_NAMED(NODE_NAME,"IssueCommand : " << ex.what());
        throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM_NAMED(NODE_NAME,"IssueCommand : ?");
        throw;
    }
    return true;
}

bool RosRoboteqDrv::ManualCANCommand(TSrvCAN_Req &req, TSrvCAN_Res &res)
{
    std::stringstream ss;

    if( _comunicator.Mode() == RoboteqCom::eCAN )
    {
        ss << "@0" << req.can_id  << "!G " << req.channel << " " << req.speed;
    }

    try
    {
        _comunicator.IssueCommand(ss.str());
        ROS_INFO_STREAM("ManualCMD= " << ss.str());
    }
    catch(std::exception& ex)
    {
        ROS_ERROR_STREAM_NAMED(NODE_NAME,"IssueCommand : " << ex.what());
        throw;
    }
    catch(...)
    {
        ROS_ERROR_STREAM_NAMED(NODE_NAME,"IssueCommand : ?");
        throw;
    }
    return true;
}

geometry_msgs::Twist RosRoboteqDrv::ConvertWheelVelocityToTwist(float left_velocity, float right_velocity)
{
    // using the two equations for left and right, we solve for long. vel and we get two equations for it. Add them together, and we end up with VL = (right - left) * r / 2 
    float longitudinal_velocity = (right_velocity - left_velocity) * (WHEEL_DIAMETER_SCIPIO / 4);

    geometry_msgs::Twist twistVelocity;

    // linear.x is just the average of left and right wheel velocities converted to linear by multiplying it by radius
    twistVelocity.linear.x = ((left_velocity + right_velocity) / 2) * (WHEEL_DIAMETER_SCIPIO / 2);

    twistVelocity.angular.z = (longitudinal_velocity * 2 * TRACK_WIDTH) / (TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE);

    return twistVelocity;
}

roboteq_node::wheels_msg RosRoboteqDrv::ConvertTwistToWheelVelocity(const geometry_msgs::Twist::ConstPtr& twist_velocity) // look into removing permanently
{
    float longitudinalVelocity = ((twist_velocity->angular.z) * (TRACK_WIDTH * TRACK_WIDTH + WHEEL_BASE * WHEEL_BASE)) / (2 * TRACK_WIDTH);

    roboteq_node::wheels_msg wheelVelocity;

    wheelVelocity.left      = -1*longitudinalVelocity + twist_velocity->linear.x;
    wheelVelocity.right     = longitudinalVelocity + twist_velocity->linear.x;

    return wheelVelocity;
}

// RoboteqCom Events
void    RosRoboteqDrv::OnMsgEvent(const IEventArgs& evt)
{
	ROS_DEBUG_STREAM_NAMED(NODE_NAME, "OnMsgEvent: " << evt.Reply());

	switch( evt.Reply()[0] )
	{
		case 'S':
			Process_S( evt );
		break;

		case 'G':
			Process_G( evt );
		break;

		case 'N':
			Process_N( evt );
		break;

		default:
		break;
			
	}
}

void	RosRoboteqDrv::Process_S(const IEventArgs& evt)
{
	try
    {
      	string::size_type idx = evt.Reply().find_first_of('=');

        if( idx != string::npos )
        {
                //idx++;
                string::size_type idy = evt.Reply().find_first_of(':', idx);

                if( idy != string::npos )
                {
                        char* pVal1 = (char*)(evt.Reply().c_str() + idx + 1);
                        char* pVal2 = (char*)(evt.Reply().c_str() + idy);

                        *pVal2 = 0L;
                        pVal2++;

                        int firstVal  = atoi( pVal1 );
                        int secondVal = atoi( pVal2 );

			            roboteq_node::wheels_msg wheelVelocity;

			            wheelVelocity.right 	= firstVal  * RPM_TO_RAD_PER_SEC;
			            wheelVelocity.left		= secondVal * RPM_TO_RAD_PER_SEC;

                        _pub.publish(RosRoboteqDrv::ConvertWheelVelocityToTwist(wheelVelocity.left, wheelVelocity.right));
                        //ROS_INFO_STREAM("Wheel RPM's: " << firstVal << " :: " << secondVal);
                }
                else
		        {
                  	ROS_ERROR_STREAM_NAMED(NODE_NAME,"Invalid(2) S Reply Format");
		        }
        }
        else
	    {
                ROS_ERROR_STREAM_NAMED(NODE_NAME,"Invalid(1) S Reply Format");
	    }
	}
	catch(std::exception& ex)
	{
		ROS_ERROR_STREAM_NAMED(NODE_NAME,"Process_S : " << ex.what());
	}
	catch(...)
	{
		ROS_ERROR_STREAM_NAMED(NODE_NAME,"Process_S : ?");
	}
}

void    RosRoboteqDrv::Process_G(const IEventArgs& evt)
{

}

void	RosRoboteqDrv::Process_N(const IEventArgs& evt)
{

}

bool    RosRoboteqDrv::IsLogOpen(void) const
{
    return _logEnabled;
}

// RoboteqCom and app Log Messages. Do append newline
void    RosRoboteqDrv::LogLine(const char* pBuffer, unsigned int len)
{
    ROS_INFO_STREAM_NAMED(NODE_NAME," - " << pBuffer);
}

// RoboteqCom and app Log Messages. Do append newline
void    RosRoboteqDrv::LogLine(const std::string& message)
{
	ROS_INFO_STREAM_NAMED(NODE_NAME," - " << message);
}

// RoboteqCom and app Log Messages. Do not append newline
void    RosRoboteqDrv::Log(const char* pBuffer, unsigned int len)
{
    ROS_INFO_STREAM_NAMED(NODE_NAME," - " << pBuffer);
}

// RoboteqCom and app Log Messages. Do not append newline
void    RosRoboteqDrv::Log(const std::string& message)
{
	ROS_INFO_STREAM_NAMED(NODE_NAME,message);
}

void    Split(TStrVec& vec, const string& str)
{
    if( str.empty() )
        return;

    string::size_type startIdx(0);

    while(true)
    {
        string::size_type startIdy = str.find_first_of(' ', startIdx);

        if( startIdy == string::npos )
        {
            vec.push_back( str.substr( startIdx ) );
            break;
        }
        else
        {
            vec.push_back( str.substr( startIdx, startIdy -  startIdx) );
            startIdx = startIdy+1;
        }
    }
}


