#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <memory.h>
#include <assert.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include "shm_com.h"
#include "semaphore.h"
#include "atris_imemory_api.h"
#include "log/log.h"

namespace shm {
/*init share memory and shm*/
static bool _memshare_init_mem_shm(shared_buf_t *buf, EN_SEM_SEED sem_seed, EN_SHM_SEED shm_seed, const char* shm_name)
{
  /*init semaphore*/
  buf->sem_id = semget((key_t)sem_seed, 1, 0666 | IPC_CREAT);
  if (buf->sem_id == -1){
    log_error("%s [%s] failed to semget.",__FUNCTION__, shm_name);
    return false;
  }
  
  if (!set_semvalue(buf->sem_id, 1)) {
    log_error("%s [%s] failed to set_semvalue.",__FUNCTION__, shm_name);
    return false;
  }

  /* Init shared-memory */
  buf->shm_id = shmget((key_t)shm_seed, sizeof(struct shared_use_st), 0666 | IPC_CREAT);
  if (buf->shm_id == -1){
    log_error("%s [%s] failed to shmget.",__FUNCTION__, shm_name);
    return false;
  }

  buf->shared_memory = shmat(buf->shm_id, (void*)0, 0);
  if (buf->shared_memory == (void*)-1){
    log_error("%s [%s] failed to shmat.",__FUNCTION__, shm_name);
    return false;
  }

  buf->shm_name = shm_name;
  buf->shared_stuff = (struct shared_use_st*)buf->shared_memory;
  buf->init_flag = 1;  
  return true;
}

//write share memory according to struct
static bool _memshare_write_data(shared_buf_t *buf, void *in_data, int size) {
  if((buf == NULL) || (in_data == NULL) || (size >= MAX_SHM_SIZE)) {
    log_error("%s [%s] buf->%p, in_data->%p, size->(%d,%d).",
      __FUNCTION__, buf->shm_name.c_str(), buf, in_data, size, MAX_SHM_SIZE);
    return false;
  }
  
  if(buf->init_flag == 0)  {
    log_error("%s [%s]  share mem is not inited.", __FUNCTION__, buf->shm_name.c_str());
    return 0;
  }
  
  semaphore_p(buf->sem_id);
  memcpy((void *)buf->shared_stuff->shm_sp, in_data, size);
  semaphore_v(buf->sem_id);
  return true;
}

//read share memory according to struct 
static bool _memshare_read_data(shared_buf_t *buf, void *out_data, int size) {
  if((buf == NULL) || (out_data == NULL) || (size >= MAX_SHM_SIZE)) {
    log_error("%s [%s] buf->%p, out_data->%p, size->(%d,%d).",
      __FUNCTION__, buf->shm_name.c_str(), buf, out_data, size, MAX_SHM_SIZE);
    return false;
  }
  if(buf->init_flag == 0)  {
    log_error("%s [%s]  share mem is not inited.", __FUNCTION__, buf->shm_name.c_str());
    return false;
  }
  semaphore_p(buf->sem_id);
  memcpy((void *)out_data, (void*)buf->shared_stuff->shm_sp, size);
  semaphore_v(buf->sem_id);
  return true;
} 

/////////////////////////////////////////////

static shared_buf_t buf_SWVersion={ 0 };

static shared_buf_t buf_UpgradeStatus={ 0 };

static shared_buf_t buf_TimeCalibrated={ 0 };

static shared_buf_t buf_RobotBraked={ 0 };

static shared_buf_t buf_UltraSound={ 0 };

static shared_buf_t buf_OdomInfo={ 0 };

static shared_buf_t buf_ChargeState={ 0 };

static shared_buf_t buf_RebootBraked={ 0 };

static shared_buf_t buf_ProtectMode={ 0 };

static shared_buf_t buf_Robot={ 0 };

static shared_buf_t buf_CompanyId={ 0 };

static shared_buf_t buf_TaskInfo={ 0 };

static shared_buf_t buf_TaskResult={ 0 };

static shared_buf_t *get_buf_SWVersion(void) {
  return &buf_SWVersion;
}

static shared_buf_t *get_buf_UpgradeStatus(void) {
  return &buf_UpgradeStatus;
}

static shared_buf_t *get_buf_TimeCalibrated(void) {
  return &buf_TimeCalibrated;
}

static shared_buf_t *get_buf_RobotBraked(void) {
  return &buf_RobotBraked;
}

static shared_buf_t *get_buf_UltraSound(void) {
  return &buf_UltraSound;
}

static shared_buf_t *get_buf_OdomInfo(void) {
  return &buf_OdomInfo;
}

static shared_buf_t *get_buf_ChargeState(void) {
  return &buf_ChargeState;
}

static shared_buf_t *get_buf_RebootBraked(void) {
  return &buf_RebootBraked;
}

static shared_buf_t *get_buf_ProtectMode(void) {
  return &buf_ProtectMode;
}

static shared_buf_t *get_buf_Robot(void) {
  return &buf_Robot;
}

static shared_buf_t *get_buf_CompanyId(void) {
  return &buf_CompanyId;
}

static shared_buf_t *get_buf_TaskInfo(void) {
  return &buf_TaskInfo;
}

static shared_buf_t *get_buf_TaskResult(void) {
  return &buf_TaskResult;
}

/*init share memory for every table */
bool iMemory_init(void) {
  memset(get_buf_SWVersion(), 0, sizeof(shared_buf_t));
  memset(get_buf_UpgradeStatus(), 0, sizeof(shared_buf_t));
  memset(get_buf_TimeCalibrated(), 0, sizeof(shared_buf_t));
  memset(get_buf_RobotBraked(), 0, sizeof(shared_buf_t));
  memset(get_buf_UltraSound(), 0, sizeof(shared_buf_t));
  memset(get_buf_OdomInfo(), 0, sizeof(shared_buf_t));
  memset(get_buf_ChargeState(), 0, sizeof(shared_buf_t));
  memset(get_buf_RebootBraked(), 0, sizeof(shared_buf_t));
  memset(get_buf_ProtectMode(), 0, sizeof(shared_buf_t));
  memset(get_buf_Robot(), 0, sizeof(shared_buf_t));
  memset(get_buf_CompanyId(), 0, sizeof(shared_buf_t));
  memset(get_buf_TaskInfo(), 0, sizeof(shared_buf_t));
  memset(get_buf_TaskResult(), 0, sizeof(shared_buf_t));

  _memshare_init_mem_shm(get_buf_SWVersion(),SEM_SEED_SWVersion,SHM_SEED_SWVersion, "SWVersion");
  _memshare_init_mem_shm(get_buf_UpgradeStatus(),SEM_SEED_UpgradeStatus,SHM_SEED_UpgradeStatus, "UpgradeStatus");
  _memshare_init_mem_shm(get_buf_TimeCalibrated(),SEM_SEED_TimeCalibrated,SHM_SEED_TimeCalibrated, "TimeCalibrated");
  _memshare_init_mem_shm(get_buf_RobotBraked(),SEM_SEED_RobotBraked,SHM_SEED_RobotBraked, "RobotBraked");
  _memshare_init_mem_shm(get_buf_UltraSound(),SEM_SEED_UltraSound,SHM_SEED_UltraSound, "UltraSound");
  _memshare_init_mem_shm(get_buf_OdomInfo(),SEM_SEED_OdomInfo,SHM_SEED_OdomInfo, "OdomInfo");
  _memshare_init_mem_shm(get_buf_ChargeState(),SEM_SEED_ChargeState,SHM_SEED_ChargeState, "ChargeState");
  _memshare_init_mem_shm(get_buf_RebootBraked(),SEM_SEED_RebootBraked,SHM_SEED_RebootBraked, "RebootBraked");
  _memshare_init_mem_shm(get_buf_ProtectMode(),SEM_SEED_ProtectMode,SHM_SEED_ProtectMode, "ProtectMode");
  _memshare_init_mem_shm(get_buf_Robot(),SEM_SEED_Robot,SHM_SEED_Robot, "Robot");
  _memshare_init_mem_shm(get_buf_CompanyId(),SEM_SEED_CompanyId,SHM_SEED_CompanyId, "CompanyId");
  _memshare_init_mem_shm(get_buf_TaskInfo(),SEM_SEED_TaskInfo,SHM_SEED_TaskInfo, "TaskInfo");
  _memshare_init_mem_shm(get_buf_TaskInfo(),SEM_SEED_TaskResult,SHM_SEED_TaskResult, "TaskResult");
  return true;
}

//write SWVersion to mem
bool iMemory_write_SWVersion(shm::SWVersion *in_info) {
  return _memshare_write_data(get_buf_SWVersion(),(void *)in_info,sizeof(shm::SWVersion));
}

//read SWVersion data to mem
bool iMemory_read_SWVersion(shm::SWVersion *out_info) {
  return _memshare_read_data(get_buf_SWVersion(),(void *)out_info,sizeof(shm::SWVersion));
}

//write UpgradeStatus to mem
bool iMemory_write_UpgradeStatus(shm::UpgradeStatus *in_info) {
  return _memshare_write_data(get_buf_UpgradeStatus(),(void *)in_info,sizeof(shm::UpgradeStatus));
}

//read UpgradeStatus data to mem
bool iMemory_read_UpgradeStatus(shm::UpgradeStatus *out_info) {
  return _memshare_read_data(get_buf_UpgradeStatus(),(void *)out_info,sizeof(shm::UpgradeStatus));
}

//write TimeCalibrated to mem
bool iMemory_write_TimeCalibrated(shm::TimeCalibrated *in_info) {
  return _memshare_write_data(get_buf_TimeCalibrated(),(void *)in_info,sizeof(shm::TimeCalibrated));
}

//read TimeCalibrated data to mem
bool iMemory_read_TimeCalibrated(shm::TimeCalibrated *out_info) {
  return _memshare_read_data(get_buf_TimeCalibrated(),(void *)out_info,sizeof(shm::TimeCalibrated));
}

//write RobotBraked to mem
bool iMemory_write_RobotBraked(shm::RobotBraked *in_info) {
  return _memshare_write_data(get_buf_RobotBraked(),(void *)in_info,sizeof(shm::RobotBraked));
}

//read RobotBraked data to mem
bool iMemory_read_RobotBraked(shm::RobotBraked *out_info) {
  return _memshare_read_data(get_buf_RobotBraked(),(void *)out_info,sizeof(shm::RobotBraked));
}

//write UltraSound to mem
bool iMemory_write_UltraSound(shm::UltraSound *in_info) {
  return _memshare_write_data(get_buf_UltraSound(),(void *)in_info,sizeof(shm::UltraSound));
}

//read UltraSound data to mem
bool iMemory_read_UltraSound(shm::UltraSound *out_info) {
  return _memshare_read_data(get_buf_UltraSound(),(void *)out_info,sizeof(shm::UltraSound));
}

//write OdomInfo to mem
bool iMemory_write_OdomInfo(shm::OdomInfo *in_info) {
  return _memshare_write_data(get_buf_OdomInfo(),(void *)in_info,sizeof(shm::OdomInfo));
}

//read OdomInfo data to mem
bool iMemory_read_OdomInfo(shm::OdomInfo *out_info) {
  return _memshare_read_data(get_buf_OdomInfo(),(void *)out_info,sizeof(shm::OdomInfo));
}

//write ChargeState to mem
bool iMemory_write_ChargeState(shm::ChargeState *in_info) {
  return _memshare_write_data(get_buf_ChargeState(),(void *)in_info,sizeof(shm::ChargeState));
}

//read ChargeState data to mem
bool iMemory_read_ChargeState(shm::ChargeState *out_info) {
  return _memshare_read_data(get_buf_ChargeState(),(void *)out_info,sizeof(shm::ChargeState));
}

//write RebootBraked to mem
bool iMemory_write_RebootBraked(shm::RebootBraked *in_info) {
  return _memshare_write_data(get_buf_RebootBraked(),(void *)in_info,sizeof(shm::RebootBraked));
}

//read RebootBraked data to mem
bool iMemory_read_RebootBraked(shm::RebootBraked *out_info) {
  return _memshare_read_data(get_buf_RebootBraked(),(void *)out_info,sizeof(shm::RebootBraked));
}

//write ProtectMode to mem
bool iMemory_write_ProtectMode(shm::ProtectMode *in_info) {
  return _memshare_write_data(get_buf_ProtectMode(),(void *)in_info,sizeof(shm::ProtectMode));
}

//read ProtectMode data to mem
bool iMemory_read_ProtectMode(shm::ProtectMode *out_info) {
  return _memshare_read_data(get_buf_ProtectMode(),(void *)out_info,sizeof(shm::ProtectMode));
}

//write Robot to mem
bool iMemory_write_Robot(shm::Robot *in_info) {
  return _memshare_write_data(get_buf_Robot(),(void *)in_info,sizeof(shm::Robot));
}

//read Robot data to mem
bool iMemory_read_Robot(shm::Robot *out_info) {
  return _memshare_read_data(get_buf_Robot(),(void *)out_info,sizeof(shm::Robot));
}

//write CompanyId to mem
bool iMemory_write_CompanyId(shm::CompanyId *in_info) {
  return _memshare_write_data(get_buf_CompanyId(),(void *)in_info,sizeof(shm::CompanyId));
}

//read CompanyId data to mem
bool iMemory_read_CompanyId(shm::CompanyId *out_info) {
  return _memshare_read_data(get_buf_CompanyId(),(void *)out_info,sizeof(shm::CompanyId));
}

//write TaskInfo to mem
bool iMemory_write_TaskInfo(shm::TaskInfo *in_info) {
  return _memshare_write_data(get_buf_TaskInfo(),(void *)in_info,sizeof(shm::TaskInfo));
}

//read TaskInfo data to mem
bool iMemory_read_TaskInfo(shm::TaskInfo *out_info) {
  return _memshare_read_data(get_buf_TaskInfo(),(void *)out_info,sizeof(shm::TaskInfo));
}

//write TaskResult to mem
bool iMemory_write_TaskResult(shm::TaskResult *in_info) {
  return _memshare_write_data(get_buf_TaskResult(),(void *)in_info,sizeof(shm::TaskResult));
}

//read TaskResult data to mem
bool iMemory_read_TaskResult(shm::TaskResult *out_info) {
  return _memshare_read_data(get_buf_TaskResult(),(void *)out_info,sizeof(shm::TaskResult));
}
}
