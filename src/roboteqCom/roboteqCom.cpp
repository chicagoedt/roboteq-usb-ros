#include "roboteqCom.h"
#include <unistd.h>
#include <string.h> // For strtok
#include <iomanip>

namespace oxoocoffee
{

#define     ROBO_TERMINATOR		'\r'
#define	    ROBO_MSG_MAX		1024

string ToHex(const string& s, bool upper_case /* = true */)
{
    ostringstream ret;

    for (string::size_type i = 0; i < s.length(); ++i)
        ret << std::setw(2) << std::setfill('0') << std::hex << (upper_case ? std::uppercase : std::nouppercase) << (int)s[i];

    return ret.str();
}

RoboteqCom::RoboteqCom(SerialLogger& log)
 : _port(log), _mode(eSerial), _event(_dummyEvent), _thread(*this)
{
    // This is just to shut up compiler warning
    // of _dummyEvent not used
    IEventArgs dummy("");
    _dummyEvent.OnMsgEvent( dummy );

    CTorInit();
}

RoboteqCom::RoboteqCom(SerialLogger& log, IRoboteqEvent& event)
 : _port(log), _mode(eSerial), _event(event), _thread(*this)
{
    CTorInit();
}

void    RoboteqCom::CTorInit(void)
{
}

void    RoboteqCom::Open(eMode mode, const string& device)
{
    _mode = mode;

    if( mode == eSerial )
        _port.logLine("RoboteqCom - connecting [SERIAL]");
    else
        _port.logLine("RoboteqCom - connecting [CAN]");

    _port.canonical(SerialPort::eCanonical_Disable);
    _port.baud(115200);
    _port.dateSize(SerialPort::eDataSize_8Bit);
    _port.stopBit(SerialPort::eStopBit_1);
    _port.parity(SerialPort::eParity_None);
    _port.flowControl(SerialPort::eFlow_None);

    _port.connect( device );

    _port.logLine("RoboteqCom - connected");

    if( IssueCommand("#") <= 0 )
    {
         _port.log("RoboteqCom - Clears out auto message responce FAILED ");
         throw std::runtime_error("RoboteqCom - Clears out auto message responce FAILED ");
    }

    if( IssueCommand("# C") <= 0 )
    {
         _port.log("RoboteqCom - Clears out telemetry strings FAILED ");
         throw std::runtime_error("RoboteqCom - Clears out telemetry strings FAILED ");
    }

    if( IssueCommand("^ECHOF 1") <= 0)
    {
         _port.log("RoboteqCom - ECHO OFF send FAILED ");
         throw std::runtime_error("RoboteqCom - ECHO OFF Send FAILED ");
    }

    if( Synchronize( ) == false )
    {
        _port.log("RoboteqCom - RoboteqCom - Synchronization Failed ^ECHOF 1");
        throw std::runtime_error("RoboteqCom - RoboteqCom - Synchronization Failed ^ECHOF 1");
    }

    if( _port.isOpen() == false )
        throw std::runtime_error("RoboteqCom - Synchronization Failed");

    if( IssueCommand("?$1E") > 0 )
    {
        _port.log("RoboteqCom - ver: ");
    
        if( ReadReply( _version ) > 0 )
        {
            string::size_type Idx = _version.find_first_of("=");

            if( Idx != string::npos)
                _version = _version.substr( Idx + 1, _version.size() - (Idx + 2));  // Strip '\r'

            _port.logLine(_version);

            if( IssueCommand("?$1F") > 0 )
            {
                _port.log("RoboteqCom - mod: ");
    
                if( ReadReply( _model ) > 0 )
                {
                    string::size_type Idx = _model.find_first_of(":");
                    
                    if( Idx != string::npos)
                        _model = _model.substr( Idx + 1, _model.size() - (Idx + 2));    // Strip '\r'

                    _port.logLine(_model);
                }
                else
                {
                    ostringstream i2a; i2a << "RoboteqCom - ERROR Model: errno " << errno;
                    _port.logLine(i2a.str());
                }
            }
            else
            {
                _port.log("RoboteqCom - checking model FAILED ");
                throw std::runtime_error("RoboteqCom - checking model FAILED ");
            }
        }
        else
        {
            ostringstream i2a; i2a << "RoboteqCom - ERROR Version: errno " << errno;
            _port.logLine(i2a.str());
        }

        _port.logLine("RoboteqCom - login ok");
    }
    else
    {
        _port.log("RoboteqCom - checking version FAILED ");
        throw std::runtime_error("RoboteqCom - checking version FAILED ");
    }

    if( _event.Type() == IRoboteqEvent::eReal )
    {
        // Running in threading mode
        _thread.Start();
        _port.logLine("RoboteqCom - reader started");
    }
}

void    RoboteqCom::Close(void)
{
    _mtx.Lock();
    if( _port.isOpen() )
        _port.disconnect();
    _mtx.UnLock();

    if( _event.Type() == IRoboteqEvent::eReal )
    {
        _port.logLine("RoboteqCom - joining reader");
        _thread.Join();
        _port.logLine("RoboteqCom - joining reader done");
    }
}

int     RoboteqCom::IssueCommand(const char* buffer, int size)
{
    return IssueCommand( string(buffer, size) );
}

int     RoboteqCom::IssueCommand(const string&  command,
                                 const string&  args)
{
    if( _thread.IsRunning() )
    {
        //RoboScopedMutex lock(_mtx);
	
        if(args == "")
            return _port.write(command + ROBO_TERMINATOR);
        else
            return _port.write(command + " " + args + ROBO_TERMINATOR);
    }
    else
    {
        if(args == "")
            return _port.write(command + ROBO_TERMINATOR);
        else
            return _port.write(command + " " + args + ROBO_TERMINATOR);
    }
}

int    RoboteqCom::ReadReply(string& reply)
{
    reply.clear();

    if( _port.Canonical() == SerialPort::eCanonical_Enable )
    {
        char buf[ROBO_MSG_MAX + 1];
        int  countRcv(0);

        if( (countRcv = _port.read(buf, ROBO_MSG_MAX)) <= 0 )
            return 0;

        buf[countRcv] = 0;
        reply.append(buf, countRcv);

        return reply.length();
    }
    else
    {
        char byte;

        while(true)
        {
            if( _port.read(&byte, 1) <= 0 )
                break;

            if( byte == ROBO_TERMINATOR )
            {
                if( reply.size() == 0 )
                    continue;

                return reply.length();
            } 

            reply.append(&byte, 1);
        }

        return 0;
    }
}

bool    RoboteqCom::Synchronize(void)
{
    char byte;

    while(true)
    {
        if( _port.read(&byte, 1) <= 0 )
            break;

        if( byte == '+' )
            return true;
    }

    return false;
}

// This methods runs on seperate thread
void    RoboteqCom::Run(void)
{
    string buffer;

    try
    {
        while( _port.isOpen() )
        {
            if( ReadReply(buffer) > 0 && buffer.size() > 0 )
            {
                if(buffer[0] != '+')
                {
                    IEventArgs evt( buffer);
                    _event.OnMsgEvent( evt );
                }
            }
        }
    }
    catch(...)
    {
        _port.logLine("RoboteqCom - reader exiting EXCEPTION");
    }

    _port.logLine("RoboteqCom - reader exiting");
}

}   // End of oxoocoffee namespace


