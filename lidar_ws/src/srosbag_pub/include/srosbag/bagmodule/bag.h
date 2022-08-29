//
// Created by lfc on 16-7-21.
//



//#info(version create time)
//startstamp
//    bagstamp
//    msgcount
//    topicname(msg1)
//      headerlength
//      header
//      body
//    topicname(msg2)
//       headerlength
//       header
//       body
//    bagstamp
//    ...
//endstamp
//-1(end)
#ifndef TESTBIN_BAG_H
#define TESTBIN_BAG_H
#define MAXCHARSIZE 100
#define srosoutstream std::cout

#include <fstream>
#include <vector>

namespace bag {
class Bag {
public:
    bool openBag(const char *path, std::ios::openmode mode);

    void closeBag();

    void wirteBaginfo(std::string bagversion, int64_t bagstamp);

    bool readBaginfo(std::string &bagversion, int64_t &bagstamp);

    void writeStamp(int64_t bagstamp);

    void readStamp(int64_t &bagstamp);

    bool writeMsgcount(int msgcount);

    bool readMsgcount(int &msgcount);

    void writeTopicname(std::string topicname);

    void readTopicname(std::string &topicname);

    void writeHeaderlength(int length);

    void readHeaderlength(int &length);

    void writeHead(char *buffer, int size);

    void readHead(char *buffer, int size);

    void writeBody(char *buffer, int size);

    void readBody(char *buffer, int size);

    void writeEnd(uint64_t endstamp);

    bool isopen();

    void writeFlush();

    int64_t getWritePosition();

    int64_t getReadPosition();

private:
    std::fstream bagfile;


};
}

#endif //TESTBIN_BAG_H
