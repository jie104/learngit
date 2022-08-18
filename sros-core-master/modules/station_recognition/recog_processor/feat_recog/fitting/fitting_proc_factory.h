//
// Created by lfc on 17-9-25.
//

#ifndef PROJECT_FITTING_PROC_FACTORY_H
#define PROJECT_FITTING_PROC_FACTORY_H

#include "fitting_processor.hpp"

namespace fitting{
class FittingProcFactory {
public:
    static FittingProcessor_Ptr getFittingProcessor(FitProcType type);
private:


};

}


#endif //PROJECT_FITTING_PROC_FACTORY_H
