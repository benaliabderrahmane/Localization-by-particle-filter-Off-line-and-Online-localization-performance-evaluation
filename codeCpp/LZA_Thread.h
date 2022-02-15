

#include <Tools/Safe_shared.hpp>
#include <datas/sensor_data.hpp>
#include <string>


#define LZA_IDLEMODE 0
#define LZA_STARTING -1
#define LZA_INITMODE 1
#define LZA_RUNINGMODE 2
#define LZA_BACKGROUNDMODE 3
#define LZA_LAS_LOGMODE 4

#define LZA_INIT_COMPASMODE 5
#define LZA_RUNING_COMMPASMODE 6

#define LZA_FUS_INITMODE 7
#define LZA_FUS_RUNINGMODE 8
#define LZA_FUS_INIT_COMPASMODE 9
#define LZA_FUS_RUNING_COMMPASMODE 10

#define LZA_TESTING 12
#define LZA_HALTINGMODE -2
#define COMPAS_IDLEMODE 0
#define COMPAS_RUNNINGMODE 1
#define COMPAS_LOGLAS 3
#define COMPAS_OBSERVEMODE 4
#define COMPAS_RTMODE 5

void LZAthread(SafeShared<euclid_position> &RecalageLoc, SafeShared<laser_4m_record> &las1, SafeShared<laser_4m_record> &las2, SafeShared<euclid_position> &odom, SafeShared<int> &ModeLZA, SafeShared<int> &ModeCompas, SafeShared<euclid_position> &LZA_res, SafeShared<double> &Compas_res, bool verbose, bool Logging);
