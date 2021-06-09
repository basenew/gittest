#ifndef __ATRIS_MEMSHARE_API_H__
#define __ATRIS_MEMSHARE_API_H__

#include "atris_param_table.h"
namespace shm {
/*init  share memory for every table */
bool iMemory_init(void);

//write SWVersion to mem
bool iMemory_write_SWVersion(shm::SWVersion *in_info);

//read SWVersion data to mem
bool iMemory_read_SWVersion(shm::SWVersion *out_info);

//write UpgradeStatus to mem
bool iMemory_write_UpgradeStatus(shm::UpgradeStatus *in_info);

//read UpgradeStatus data to mem
bool iMemory_read_UpgradeStatus(shm::UpgradeStatus *out_info);

//write TimeCalibrated to mem
bool iMemory_write_TimeCalibrated(shm::TimeCalibrated *in_info);

//read TimeCalibrated data to mem
bool iMemory_read_TimeCalibrated(shm::TimeCalibrated *out_info);

//write RobotBraked to mem
bool iMemory_write_RobotBraked(shm::RobotBraked *in_info);

//read RobotBraked data to mem
bool iMemory_read_RobotBraked(shm::RobotBraked *out_info);

//write UltraSound to mem
bool iMemory_write_UltraSound(shm::UltraSound *in_info);

//read UltraSound data to mem
bool iMemory_read_UltraSound(shm::UltraSound *out_info);

//write OdomInfo to mem
bool iMemory_write_OdomInfo(shm::OdomInfo *in_info);

//read OdomInfo data to mem
bool iMemory_read_OdomInfo(shm::OdomInfo *out_info);

//write ChargeState to mem
bool iMemory_write_ChargeState(shm::ChargeState *in_info);

//read ChargeState data to mem
bool iMemory_read_ChargeState(shm::ChargeState *out_info);

//write RebootBraked to mem
bool iMemory_write_RebootBraked(shm::RebootBraked *in_info);

//read RebootBraked data to mem
bool iMemory_read_RebootBraked(shm::RebootBraked *out_info);

//write ProtectMode to mem
bool iMemory_write_ProtectMode(shm::ProtectMode *in_info);

//read ProtectMode data to mem
bool iMemory_read_ProtectMode(shm::ProtectMode *out_info);

//write Robot to mem
bool iMemory_write_Robot(shm::Robot *in_info);

//read Robot data to mem
bool iMemory_read_Robot(shm::Robot *out_info);

//write CompanyId to mem
bool iMemory_write_CompanyId(shm::CompanyId *in_info);

//read CompanyId data to mem
bool iMemory_read_CompanyId(shm::CompanyId *out_info);

//write TaskInfo to mem
bool iMemory_write_TaskInfo(shm::TaskInfo *in_info);

//read TaskInfo to mem
bool iMemory_read_TaskInfo(shm::TaskInfo *out_info);

//write TaskResult to mem
bool iMemory_write_TaskResult(shm::TaskResult *in_info);

//read TaskResult to mem
bool iMemory_read_TaskResult(shm::TaskResult *out_info);

}

#endif
