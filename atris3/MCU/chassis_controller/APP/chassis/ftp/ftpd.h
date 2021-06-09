#ifndef _FTPD_H
#define _FTPD_H


#ifdef __cplusplus
extern "C" {
#endif

#ifndef FTPD_DEBUG
#define FTPD_DEBUG LWIP_DBG_OFF
#endif

#define FTPD_PORT 21 // FTP�������˿ں�
#define FTPD_PASV 1 // �Ƿ�����ʹ��PASV����

// ���������е��쳣���
#define FTPD_CMDSTEP_CONNFAILED 0x1000 // ���ӽ���ʧ��
#define FTPD_CMDSTEP_CONNABORTED 0x2000 // ���ӽ����ɹ����쳣��ֹ
#define FTPD_CMDSTEP_CONNSHUTDOWN 0x4000 // ���ӽ����ɹ��󱻿ͻ��˹ر�

// FTP�ͻ���״̬λ
#if FTPD_PASV
#define FTPD_FLAG_PASSIVE 0x01 // ��ǰ�Ƿ�Ϊ����ģʽ
#endif
#define FTPD_FLAG_CLOSE 0x02 // �յ��ͻ��˵�TCP��һ�λ��ֺ�, �����͵����λ���
#define FTPD_FLAG_SHUTDOWN 0x04 // ������TCP��һ�λ���, Ȼ����տͻ��˵����λ���
#define FTPD_FLAG_RENAME 0x08 // �Ƿ������������ļ�
#define FTPD_FLAG_AGAIN 0x10 // ��ǰFTP���û��ִ�����, ���������ϵ����ݷ�����Ϻ�Ӧ������������
#define FTPD_FLAG_NEWDATACONN 0x20 // ���������Ѵ�������δ������
#define FTPD_FLAG_TCPERROR 0x40 // TCP�������ݳ���

// �������ӹرշ�ʽ
#define FTPD_FREEDATA_ABORT 0 // ǿ����ֹ��������
#define FTPD_FREEDATA_CLOSE 1 // �ر��������� (�ͻ����ѹر�)
#define FTPD_FREEDATA_SHUTDOWN 2 // �ر��������� (�ͻ���δ�ر�)

#ifndef MAX_PATH
#define MAX_PATH 260
#endif

struct ftpd_user
{
  char *name;
  char *password;
};

struct ftpd_account
{
  struct ftpd_user user;
  char *rootpath;
};

#ifdef FF_DEFINED
struct ftpd_state
{
  struct tcp_pcb *ctrlconn;
  struct tcp_pcb *dataconn;
  int dataport;
  
  char cmd[MAX_PATH + 20];
  int cmdlen;
  char *cmdarg;
  int cmdstep;
  char last;

  char type;
  char path[MAX_PATH];
  char rename[MAX_PATH];
  
  struct ftpd_user user;
  int userid;
  int flags;
  
  int sent; // δ�յ�ȷ�ϵ��ѷ����ֽ���
  struct pbuf *queue; // ���ݽ��ն���
  
  void *dataout;
  int dataout_len;
  DIR *dp;
  FIL *fp;
  FILINFO *finfo;
};
#else
struct ftpd_state;
#endif

int ftpd_concat_path(char *buffer, int bufsize, const char *filename);
int ftpd_file_exists(const char *path);
#ifdef FF_DEFINED
time_t ftpd_filetime(WORD fdate, WORD ftime, struct tm *ptm);
#endif
int ftpd_fullpath(const struct ftpd_state *state, char *buffer, int bufsize, const char *filename, char **puserpath);

void *ftpd_memrchr(const void *s, int c, size_t n);
int ftpd_simplify_path(char *path, int basepos);
char *ftpd_strdup(const char *s);
int ftpd_init(void);
void ftp_file_init(void);
const struct fsdata_file *get_motor_struct(const char * name);
#ifdef __cplusplus
}
#endif
#endif
