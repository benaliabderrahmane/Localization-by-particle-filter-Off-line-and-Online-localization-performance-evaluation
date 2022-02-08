#include <datas/location_data.hpp>
#include <pfoa/path_finding.hpp>
#include <Tools/Safe_shared.hpp>
#include <datas/sensor_data.hpp>
#include <vector>


#define P3D_IDLEMODE 0
#define P3D_INITMODE -1
#define P3D_STARTMODE 1
#define P3D_SMZUSMODE 2
#define P3D_SMZLASMODE 3
#define P3D_SMZLAS2MODE 4
#define P3D_SMZLASUSMODE 5
#define P3D_SMZLAS2USMODE 6
#define P3D_CENTERUSMODE 7
#define P3D_CENTERLASMODE 8
#define P3D_CENTERLAS2MODE 9
#define P3D_CENTERLASUSMODE 10
#define P3D_CENTERLAS2USMODE 11
#define P3D_TESTING 12
#define P3D_TURNING 13
//#define P3D_TURN90RIGHT 14
#define P3D_NOOAMODE 15
#define P3D_ENDINGMODE 16
#define P3D_FREEZING 17
#define P3D_BUMPER_ERRORMODE -2
#define P3D_INVALIDMOD_ERRORMODE -3
#define P3D_HALTMOD -4

void PioneerThread(SafeShared<int> &Mode, SafeShared<euclid_position> &Odopos, SafeShared<euclid_position> &Recalage, SafeShared<laser_4m_record> &releveLas1, SafeShared<laser_4m_record> &releveLas2, SafeShared<double> &vit, bool verbose, bool Logging, SafeShared<std::vector<pfoa::waypoint>> &Path);
