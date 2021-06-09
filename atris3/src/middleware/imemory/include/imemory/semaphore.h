#ifndef __SEMAPHORE_H__
#define __SEMAPHORE_H__

#define BASE_SEM_SEED 2000
typedef enum {
  SEM_SEED_sqlite = BASE_SEM_SEED,
  SEM_SEED_SWVersion,
  SEM_SEED_UpgradeStatus,
  SEM_SEED_TimeCalibrated,
  SEM_SEED_RobotBraked,
  SEM_SEED_UltraSound,
  SEM_SEED_OdomInfo,
  SEM_SEED_ChargeState,
  SEM_SEED_RebootBraked,
  SEM_SEED_ProtectMode,
  SEM_SEED_Robot,
  SEM_SEED_CompanyId,
  SEM_SEED_TaskInfo,
  SEM_SEED_TaskResult,
} EN_SEM_SEED;

union semun {
  int val;
  struct semid_ds *buf;
  unsigned short *array;
};

bool set_semvalue(int sem_id, int value);
void del_semvalue(int sem_id);
bool semaphore_p(int sem_id);
bool semaphore_v(int sem_id);

#endif
