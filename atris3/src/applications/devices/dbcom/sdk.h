#ifndef SDK_H__
#define SDK_H__

#define SDK_PORT 8728

struct sdk_sentence {
	char **word;
	int words;
};

struct sdk_result {
	struct sdk_sentence *sentence;
	char done;
	char re;
	char trap;
	char fatal;
};

struct sdk_event {
	char tag[100];
	void (*callback)(struct sdk_result *result);
	char inuse;
};

enum sdk_type {
		sdk_SIMPLE,
		sdk_EVENT
};


struct sdk_connection {
	enum sdk_type type;
#ifdef _WIN32
	SOCKET socket;
#else
	int socket;
#endif
	unsigned char *buffer;
	struct sdk_event **events;
	int max_events;
	struct sdk_result *event_result;
	int expected_length;
	int length;
};

#ifdef __cplusplus
extern "C" 
{
#endif
/* event based functions */
int sdk_send_command(struct sdk_connection *conn, char *command, ...);
void sdk_set_type(struct sdk_connection *conn, enum sdk_type type);
int sdk_runloop_once(struct sdk_connection *conn, void (*callback)(struct sdk_result *result));
int sdk_send_command_cb(struct sdk_connection *conn, void (*callback)(struct sdk_result *result), char *command, ...);
int sdk_send_sentence_cb(struct sdk_connection *conn, void (*callback)(struct sdk_result *result), struct sdk_sentence *sentence);

/* blocking functions */
struct sdk_result *sdk_send_command_wait(struct sdk_connection *conn, char *command, ...);
//struct sdk_result *sdk_send_command_wait_for_cb(struct sdk_connection *conn,char *command,..);
struct sdk_result *sdk_read_packet(struct sdk_connection *conn);
int sdk_login(struct sdk_connection *conn, char *username, char *password);
int sdk_cancel(struct sdk_connection *conn, int id);

/* common functions */
struct sdk_connection *sdk_connect(char *address, int port);
int sdk_disconnect(struct sdk_connection *conn);
void sdk_result_free(struct sdk_result *result);
char *sdk_get(struct sdk_result *result, char *key);
char *sdk_get_tag(struct sdk_result *result);

/* sentence functions */
struct sdk_sentence *sdk_sentence_new();
void sdk_sentence_free(struct sdk_sentence *sentence);
void sdk_sentence_add(struct sdk_sentence *sentence, char *word);

#ifdef __cplusplus
}
#endif

#endif