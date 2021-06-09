#include "navigateAbility.h"
#include <signal.h>
#include <thread>
#include "uWS.h"
#include "shttpd.h"
#include "tiny_ros/ros.h"
#include "log/log.h"

static void show_nav_ability(struct shttpd_arg *arg) {
    Json::FastWriter jwriter; Json::Value reponse;
    std::string request_method = shttpd_get_env(arg, "REQUEST_METHOD") ? shttpd_get_env(arg, "REQUEST_METHOD") : "";
    if (!strcmp(request_method.c_str(), "POST")) {
        if (arg->flags & SHTTPD_MORE_POST_DATA) {
            return;
        }
        std::string resp;
        if (arg->in.buf && arg->in.len > 0) {
            std::string request = std::string(arg->in.buf, arg->in.len);
            NavigateAbility::get_instance()->messageInstantReceive(request, resp);
        }
        shttpd_printf(arg, "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n");
        shttpd_printf(arg, "%s",resp.c_str());
        arg->flags |= SHTTPD_END_OF_OUTPUT;
    } else {
        shttpd_printf(arg, "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n");
        shttpd_printf(arg, "Unsupported request method: \"%s\"", request_method.c_str());
        arg->flags |= SHTTPD_END_OF_OUTPUT;
    }
}


static void web_service_run(int web_server_port) {
    try {
        uWS::Server server(web_server_port);
        server.onConnection([](uWS::WebSocket socket) {
            log_info("WebService onConnection ok!");
            int64_t *id = new int64_t;
            *id = NavigateAbility::get_instance()->addWSSession(socket);
            socket.setData(id);
        });

        server.onMessage([](uWS::WebSocket socket, char *message, std::size_t length, uWS::OpCode opCode) {
            if (opCode == uWS::TEXT) {
                std::string resp;
                std::string request = std::string(message, length);
                NavigateAbility::get_instance()->messageInstantReceive(request, resp);
                socket.send((char*)resp.data(), resp.size(), uWS::TEXT);
            }
        });

        server.onDisconnection([](uWS::WebSocket socket, int code, char *message, std::size_t length) {
            void *id = socket.getData();
            if (id) {
                NavigateAbility::get_instance()->removeWSSession(*((int64_t*)id));
                socket.setData(nullptr);
                delete ((int64_t*)id);
            }
            std::string msg = std::string(message, length);
            log_warn("WebService onDisconnection %s(%d)", msg.c_str(), code);
        });

        server.run();
    } catch (...) {
        log_error("WebService error: %s(errno: %d)", strerror(errno), errno);
    }
}


static void http_service_run(int argc, char *argv[], const char* http_server_port) {
    struct shttpd_ctx *ctx;
retry_shttpd:
    ctx = shttpd_init(argc, argv);
    if(!ctx) return;
	shttpd_set_option(ctx, "ports", http_server_port);
    shttpd_register_uri(ctx, "/navability", &show_nav_ability, NULL);
    for (;;) {
        if (shttpd_poll(ctx, 1000) < 0) {
            break;
        }
    }
    shttpd_fini(ctx);
    goto retry_shttpd;
}


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "navability");
    tinyros::init("navability");
    signal(SIGPIPE, SIG_IGN);
    
    NavigateAbility::get_instance();

    std::thread tidws(std::bind(web_service_run, 12528));
    tidws.detach();

    std::thread tidhttp(std::bind(http_service_run, argc, argv, "12538"));
    tidhttp.detach();

    ros::MultiThreadedSpinner s(10);
    ros::spin(s);

    return 0;
}

