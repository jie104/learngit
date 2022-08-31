#include "async_client.h"

Async_Client::Async_Client(fundata_t fundata_ )
    :m_fundata(fundata_)
{
    m_bConnected = false;
    m_bReconnecting = false;
    m_pSocket = boost::shared_ptr<ip::tcp::socket>(new ip::tcp::socket(m_io));
    cout << "TCP-Connection is initialized!" << endl;
}

Async_Client::~Async_Client()
{
    disconnect();
    m_fundata = NULL;
}

bool Async_Client::connect(string ip, int port) 
{
    try
    {
        if (m_bConnected)
        {
            return false;
        }
        if(m_pSocket->is_open())
        {
            boost::system::error_code errorcode;
            m_pSocket->shutdown(ip::tcp::socket::shutdown_both, errorcode);
            m_pSocket->close();
        }
        m_sServerIp = ip;
        m_iServerPort = port;
        m_ep = ip::tcp::endpoint(ip::address::from_string(ip), port);
        m_pSocket->connect(m_ep,ec);
        if(ec)
        {
            m_bConnected = false;
            return false;
        }
        else
        {
            m_bConnected = true;
            boost::thread recvThread(boost::bind(&Async_Client::recvData,this));
            return true;
        }
    }
    catch (boost::exception &e)
    {
        return false;
    }
}

bool Async_Client::disconnect() 
{
    try
    {
        cout << "Disconnecting connection!" << endl;
        m_bConnected = false;
        if(m_pSocket->is_open())
        {
            boost::system::error_code errorcode;
            m_pSocket->shutdown(ip::tcp::socket::shutdown_both, errorcode);
            m_pSocket->close();
        }
        return true;
    }
    catch(boost::exception& e)
    {
        return false;
    }
}

void Async_Client::recvData()
{
    try
    {
        while (m_bConnected)
        {
            unsigned char buf[10240];
            size_t len = m_pSocket->read_some(buffer(buf));
            if (len > 0)
            {
                (*m_fundata)(this->m_ep.address().to_string().c_str(),this->m_ep.port(),buf,len);
            }
        }
    }
    catch (std::exception e)
    {
    }
    m_bConnected = false;
}

void Async_Client::reconnect()
{
    m_bReconnecting = true;
    while (!m_bConnected)
    {
        if (m_pSocket->is_open())
        {
            boost::system::error_code errorcode;
            m_pSocket->shutdown(ip::tcp::socket::shutdown_both, errorcode);
            m_pSocket->close();
            cout << "Initializing network!" << endl;
        }
        sleep(3);
        cout << "Start reconnecting laser!" << endl;
        sleep(2);
        if (connect(m_sServerIp, m_iServerPort))
        {
            cout << "Succesfully connected!" << endl;
            break;
        }
        else
        {
            cout << "Failed to reconnect!" << endl;
        }
        sleep(2);
    }
    m_bReconnecting = false;
}

bool Async_Client::sendMsg(unsigned char buf[], int length)
{
    try
    {
        if (m_pSocket->is_open() && m_bConnected)
        {
            size_t st = m_pSocket->send(buffer(buf, length));
            if (st == length)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    catch (std::exception e)
    {
        return false;
    }
}
