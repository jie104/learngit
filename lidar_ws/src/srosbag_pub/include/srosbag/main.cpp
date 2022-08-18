#include <iostream>
#include "bagmodule/bag_record.h"
#include "bagmodule/bag_play.h"
using namespace std;

int main() {
//    bag::BagRecord record;
//    record.doRecord("firstrecord");
//    record.recordStop();
    bag::BagPlay play;
    play.doPlay("/sros/uhd-test");
    while(1) {
        sleep(1);
    }

    return 0;
}