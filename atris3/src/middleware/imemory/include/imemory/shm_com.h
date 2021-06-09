#ifndef __SHM_COM_H__
#define __SHM_COM_H__
#include <string>

#define BASE_SHM_SEED (6001)
#define MAX_SHM_SIZE  (512)

typedef enum {
  SHM_SEED_SWVersion = BASE_SHM_SEED,
  SHM_SEED_UpgradeStatus,
  SHM_SEED_TimeCalibrated,
  SHM_SEED_RobotBraked,
  SHM_SEED_UltraSound,
  SHM_SEED_OdomInfo,
  SHM_SEED_ChargeState,
  SHM_SEED_RebootBraked,
  SHM_SEED_ProtectMode,
  SHM_SEED_Robot,
  SHM_SEED_CompanyId,
  SHM_SEED_TaskInfo,
  SHM_SEED_TaskResult,
} EN_SHM_SEED;

typedef struct shared_use_st {
  char shm_sp[MAX_SHM_SIZE];
} shared_use_st;

typedef struct {
  int init_flag;
  int sem_id; //semaphore id
  int shm_id; // shared-memory id
  std::string shm_name;
  void* shared_memory;
  shared_use_st* shared_stuff;
} shared_buf_t;
#endif
