This module is mainly responsible for AGV perception. At present, it mainly foc
uses on the perception of the safety of fetch and put goods.


The file organization of the module is described below:

cmake           This folder is mainly used to manage the version information of
                the perception module.

common          This folder mainly contains some public data structures, public
                classes and public function functions, such as log, point2d, po
                int3d, normal and other data structures.

lib             This is the perception module library file. All the perception
                related processing source files are in it.

perception-sros    Sensing module library file external calling program source
                   file directory.

Note:
    If you want to add static object detect module, you should inherit the Dete
    ctorModuleBase class which in the perception-sros/module_base directory. th
    en,You also need to define the instructions(eg: PALLET_QUERY_COMMAND) for c
    all and the topics for handling the result publish(eg: PALLET_INFO).In addi
    tion, all the virtual functions in the base class are implemented, such as
    the callback function of message instruction (such as onIMGMsg()) that calls
    the functions from the other module, and the main processing function run()
    is implemented according to the requirements. Finally, the processing resul
    ts are published to the predefined topic.
