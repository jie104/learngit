//
// Created by lfc on 16-7-21.
//

#include <string.h>
#include <iostream>
#include "bag.h"

namespace bag {
bool Bag::openBag(const char *path, std::ios::openmode mode) {
    bagfile.open(path, mode);
    if (bagfile.is_open()) {
        return true;
    }
    return false;
}

void Bag::closeBag() {
    bagfile.close();
}

void Bag::wirteBaginfo(std::string bagversion, int64_t time) {
    if (bagfile.is_open()) {
        bagfile.put('#');
        bagfile.write(bagversion.c_str(), strlen(bagversion.c_str()));
        bagfile.put('\n');
        bagfile.write((char *) (&time), sizeof(time));
        bagfile.put('\n');
    }
}

bool Bag::readBaginfo(std::string &bagversion, int64_t &bagtime) {
    if (bagfile.is_open()) {
        if (bagfile.get() == '#') {
            char buffer[MAXCHARSIZE];
            bagfile.getline(buffer, MAXCHARSIZE);
            bagversion = buffer;
            char buffer_time[sizeof(bagtime)];
            bagfile.read(buffer_time, sizeof(bagtime));
            bagfile.get();
            memcpy(&bagtime, buffer_time, sizeof(bagtime));
            return true;
        }

    }
    return false;
}

void Bag::writeStamp(int64_t bagstamp) {
    if (bagfile.is_open()) {
        bagfile.write((char *) (&bagstamp), sizeof(bagstamp));
        bagfile.put('\n');
    }
}

void Bag::readStamp(int64_t &bagstamp) {
    if (bagfile.is_open()) {
        char buffer_time[sizeof(bagstamp)];
        bagfile.read(buffer_time, sizeof(bagstamp));
        bagfile.get();
        memcpy(&bagstamp, buffer_time, sizeof(bagstamp));
    }
}

bool Bag::writeMsgcount(int msgcount) {
    if (bagfile.is_open()) {
        bagfile.write((char *) (&msgcount), sizeof(msgcount));
        bagfile.put('\n');
    }
    return false;
}

bool Bag::readMsgcount(int &msgcount) {
    if (bagfile.is_open()) {
        char buffer_time[sizeof(msgcount)];
        bagfile.read(buffer_time, sizeof(msgcount));
        bagfile.get();
        memcpy(&msgcount, buffer_time, sizeof(msgcount));
        if (msgcount >= 0) {
            return true;
        }
    }
    return false;
}

void Bag::writeTopicname(std::string topicname) {
    if (bagfile.is_open()) {
        bagfile.write(topicname.c_str(), strlen(topicname.c_str()));
        bagfile.put('\n');
    }

}

void Bag::readTopicname(std::string &topicname) {
    if (bagfile.is_open()) {
        char buffer[MAXCHARSIZE];
        bagfile.getline(buffer, MAXCHARSIZE);
        topicname = buffer;
    }

}

void Bag::writeHeaderlength(int length) {
    if (bagfile.is_open()) {
        bagfile.write((char *) (&length), sizeof(length));
        bagfile.put('\n');
    }

}

void Bag::readHeaderlength(int &length) {
    if (bagfile.is_open()) {
        char buffer_time[sizeof(length)];
        bagfile.read(buffer_time, sizeof(length));
        bagfile.get();
        memcpy(&length, buffer_time, sizeof(length));
    }

}

void Bag::writeHead(char *buffer, int size) {
    if (bagfile.is_open()) {
        bagfile.write(buffer, size);
        bagfile.put('\n');
    }

}

void Bag::readHead(char *buffer, int size) {
    if (bagfile.is_open()) {
        bagfile.read(buffer, size);
        bagfile.get();
    }

}

void Bag::writeBody(char *buffer, int size) {
    if (bagfile.is_open()) {
        bagfile.write(buffer, size);
        bagfile.put('\n');
    }

}

void Bag::readBody(char *buffer, int size) {
    if (bagfile.is_open()) {
        bagfile.read(buffer, size);
        bagfile.get();
    }

}

void Bag::writeEnd(uint64_t endstamp) {
    if (bagfile.is_open()) {
        bagfile.write((char *) (&endstamp), sizeof(endstamp));
        bagfile.put('\n');
        int i = -1;
        bagfile.write((char *) (&i), sizeof(i));
        bagfile.put('\n');
    }
}

bool Bag::isopen() {
    return bagfile.is_open();
}


void Bag::writeFlush() {
    bagfile << std::flush;
}

int64_t Bag::getWritePosition() {
    return bagfile.tellp();
}

int64_t Bag::getReadPosition() {
    return bagfile.tellg();
}
}