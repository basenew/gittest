/*
 * \File
 * semaphore.c
 * \Breif
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/sem.h>
#include "semaphore.h"
#include "log/log.h"

/* init semaphore by semctl */
bool set_semvalue(int sem_id, int value) {
  union semun sem_union;
  sem_union.val = value;
  if (semctl(sem_id, 0, SETVAL, sem_union) == -1) {
    return false;
  }
  return true;
}

/* delete semaphore by sectl */
void del_semvalue(int sem_id) {
  union semun sem_union;
  if (semctl(sem_id, 0, IPC_RMID, sem_union) == -1) {
    log_error("%s Failed to delete semaphore", __FUNCTION__);
  }
}

/* P(v) */
bool semaphore_p(int sem_id) {
  struct sembuf sem_b;
  sem_b.sem_num = 0;
  sem_b.sem_op = -1; /* P(v) */
  sem_b.sem_flg = SEM_UNDO;
  if (semop(sem_id, &sem_b, 1) == -1) {
    log_error("%s semaphore_p failed", __FUNCTION__);
    return false;
  }

  return true;
}

/* V(v) */
bool semaphore_v(int sem_id) {
  struct sembuf sem_b;

  sem_b.sem_num = 0;
  sem_b.sem_op = 1; // V(v)
  sem_b.sem_flg = SEM_UNDO;

  if (semop(sem_id, &sem_b, 1) == -1) {
    log_error("%s semaphore_v failed", __FUNCTION__);
    return false;
  }
  return true;
}
