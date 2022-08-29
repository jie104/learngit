#ifndef ASYNC_CLIENT_H
#define ASYNC_CLIENT_H
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <string>
#include <unistd.h>

using namespace std ;
using boost::asio::ip::udp;
using boost::asio::ip::tcp;
using namespace boost::asio;
#define MAX_LENGTH 50000
typedef void (*fundata_t)(const char* addr,int port,unsigned char* data,const int len);
class Async_Client
{
public:
  //Async_Client();
  Async_Client(fundata_t fundata_ );
  ~Async_Client();
  bool connect(string ip, int port);
  bool disconnect();
  void recvData();
  void reconnect();
  bool sendMsg(unsigned char buf[], int length); 

  bool        m_bConnected;
  bool        m_bReconnecting;

private:
  string m_sServerIp;
  int m_iServerPort;
  io_service m_io;
  ip::tcp::endpoint m_ep;
  boost::shared_ptr<ip::tcp::socket> m_pSocket;
  boost::system::error_code ec;

  char        data_[MAX_LENGTH];
  fundata_t   m_fundata ;
};

#endif // ASYNC_CLIENT_H
