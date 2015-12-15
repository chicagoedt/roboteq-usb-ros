#include <iostream>
#include <fstream>
#include <vector>
#include <unistd.h>
#include <signal.h>
#include "roboteqCom.h"

using namespace oxoocoffee;

#define LOG_FILE_NAME       "roboteqCom.log"

class RoboteqLogger : public SerialLogger
{
    public:
        RoboteqLogger(void);

        bool    Open(const string& filePath, bool threded);
        void    Close(void);

        virtual void    LogLine(const char* pBuffer, unsigned int len);
        virtual void    LogLine(const std::string& message);
		
		// DO NOT Write new line at end
        virtual void    Log(const char* pBuffer, unsigned int len);
        virtual void    Log(const std::string& message);

        virtual bool    IsLogOpen(void) const
        {
            return _file.is_open();
        }

    private:
        ofstream        _file;
        bool            _threaded;
        pthread_mutex_t _mx;
};

class RoboteqTest : public IEventListener<const IEventArgs>
{
    public:
                 RoboteqTest(void);
        virtual ~RoboteqTest(void);

        bool    Initialize(int argc, char* argv[]);
        void    Run(void);
        void    Shutdown(void);

    protected:
        // RoboteqCom Events
        virtual void OnMsgEvent(const IEventArgs& evt);

    private:
        RoboteqLogger   _logger;
        RoboteqCom      _comunicator;
        RoboMutex       _mutex;            // Optionally used if RoboteqCom setup in threaded mode
        string          _device;
};

RoboteqTest app;

void    PrintHelp(string progName);

static void SigInt(int sig)
{
    if( sig == SIGINT)
        app.Shutdown();
}

int main(int argc, char* argv[])
{
    if( argc == 1 )
    {
        PrintHelp(argv[0]);
        return 0;
    }

    signal(SIGINT,   SigInt);

    try
    {
        if( app.Initialize(argc, argv) )
            app.Run();
    }
    catch(std::exception& ex)
    {
        cout << "Exception: " << ex.what() << endl;
    }
    catch(...)
    {
        cout << "Exception: General";
    }

    return 0;
}

void    PrintHelp(string progName)
{
    string::size_type Idx = progName.find_last_of("\\/");

    if( Idx != string::npos )
        progName = progName.substr( Idx + 1 );

    cout << endl;
    cout << "Usage: " << progName << " [options]" << endl;
    cout << "   -p /dev/tty*  - serial device" << endl;
    cout << "   -m [s|c]      - [s]erial or [c]an mode" << endl;
    cout << "   -h            - this information" << endl;
}

RoboteqTest::RoboteqTest(void)
 : _comunicator(_logger, *this)
{

}

RoboteqTest::~RoboteqTest(void)
{

}

bool    RoboteqTest::Initialize(int argc, char* argv[])
{
    if( argc == 1 )
    {
        PrintHelp(argv[0]);
        return false;
    }

    int                c;
    RoboteqCom::eMode  mode = RoboteqCom::eSerial;
    string             filePath;

    while( (c = getopt( argc, argv, "p:m:h")) != -1 )
    {
        switch( c )
        {
            case 'p':
                _device = optarg;
                break;

            case 'm':
                if( optarg[0] == 's')
                    mode = RoboteqCom::eSerial;
                else if( optarg[0] == 'c')
                    mode = RoboteqCom::eCAN;
                else
                {
                    cout << "Error: invalid -m parameter: " << optarg << endl;
                    return false;
                }


                break;

            case 'h':
                PrintHelp(argv[0]);
                return false;
            break;

            default:
                break;
        }
    }

    if( _device.empty() )
    {
        cout << "Error: missing device path" << endl;
        return false;
    }

    if( _logger.Open(LOG_FILE_NAME, _comunicator.IsThreaded() ) == false )
        THROW_RUNTIME_ERROR(string("Failed to open ") + LOG_FILE_NAME);

    _comunicator.Open( mode, _device );

    _comunicator.IssueCommand("^ECHOF 1");
    _comunicator.IssueCommand("?S");    // Query for speed and enters this speed
                                        // request into telemetry system
    _comunicator.IssueCommand("# 500"); // auto message responce is 200ms

    return true;
}

void    RoboteqTest::Run(void)
{
    while( _comunicator.IsThreadRunning() )
    {
        sleep(500);
    }
}

void    RoboteqTest::Shutdown(void)
{
    _comunicator.Close();
}

// RoboteqCom Events
void RoboteqTest::OnMsgEvent(const IEventArgs& evt)
{
    cout << "> : " << evt.Reply() << endl;
    _logger.LogLine("> : " + evt.Reply());
}

RoboteqLogger::RoboteqLogger(void)
{
     pthread_mutex_init(&_mx, NULL);
}

bool    RoboteqLogger::Open(const string& filePath, bool threded)
{
    _threaded = threded;

    Close();

    _file.open( filePath.c_str(), ios_base::out | ios_base::app );

    if( _file.is_open() )
    {
        _file << "+++++++++ Opened ++++++++" << endl;
        return true;
    }
    else
        return false;
}

void    RoboteqLogger::Close(void)
{
    if( _file.is_open() )
    {
        _file << "--------- Closed --------" << endl;
        _file.close();
    }
}

void    RoboteqLogger::LogLine(const char* pBuffer, unsigned int len)
{
    if( _file.is_open() )
    {
        if( _threaded )
            pthread_mutex_lock(&_mx);

        _file.write(pBuffer, len) << std::endl;
        _file.flush();

        if( _threaded )
            pthread_mutex_unlock(&_mx);
    }
}

void    RoboteqLogger::LogLine(const std::string& message)
{
    if( _file.is_open() )
    {
        if( _threaded )
            pthread_mutex_lock(&_mx);

        _file << message << std::endl;
        _file.flush();

        if( _threaded )
            pthread_mutex_unlock(&_mx);
    }
}

// DO NOT Write new line at end
void    RoboteqLogger::Log(const char* pBuffer, unsigned int len)
{
    if( _file.is_open() )
    {
        if( _threaded )
            pthread_mutex_lock(&_mx);

        _file.write(pBuffer, len);
        _file.flush();
    
        if( _threaded )
            pthread_mutex_unlock(&_mx);
    }
}

void    RoboteqLogger::Log(const std::string& message)
{
    if( _file.is_open() )
    {
         if( _threaded )
            pthread_mutex_lock(&_mx);

        _file << message;
        _file.flush();

        if( _threaded )
            pthread_mutex_unlock(&_mx);
    }
}


