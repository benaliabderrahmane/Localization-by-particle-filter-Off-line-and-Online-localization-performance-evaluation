#include <datas/location_data.hpp>
#include <Tools/Safe_shared.hpp>
#include <datas/sensor_data.hpp>
#include <vector>

#define PF_IDLEMODE 0
#define PF_INITMODE 1
#define PF_RUNMODE 2
#define PF_HALTMODE -1


//With US
//void PFThread(SafeShared<int> &Mode, SafeShared<euclid_position> &Odopos, SafeShared<laser_4m_record> &releveLas1, SafeShared<laser_4m_record> &releveLas2, SafeShared<std::vector<struct polar_coordinate>> &US, bool verbose, bool Logging,SafeShared<euclid_position> &Estimation);
//Without US
void PFThread(SafeShared<int> &Mode, SafeShared<euclid_position> &Odopos, SafeShared<laser_4m_record> &releveLas1, SafeShared<laser_4m_record> &releveLas2, bool verbose, bool Logging,SafeShared<euclid_position> &Estimation);
