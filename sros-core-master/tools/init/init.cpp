//
// Created by lhx on 17-12-27.
//

#include <iostream>
#include <string>

#include "SQLiteCpp/Database.h"

using namespace std;

int main() {
    SQLite::Database db("test.db", SQLite::OPEN_READWRITE | SQLite::OPEN_CREATE);
//    db.exec("SELECT * FROM `table`;");
    std::cout << "Hello" << std::endl;

    std::string SN;


    cout << "序列号（SN）: ";
    std::cin >> SN;

    cout << "SN: " << SN << endl;
}
