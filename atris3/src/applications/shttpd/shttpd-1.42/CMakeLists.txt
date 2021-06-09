cmake_minimum_required(VERSION 2.6)
include_directories(src)
add_library (shttpd SHARED 
  src/string.c
  src/shttpd.c
  src/log.c
  src/auth.c
  src/md5.c
  src/cgi.c
  src/io_ssi.c
  src/io_file.c
  src/io_socket.c
  src/io_ssl.c
  src/io_emb.c
  src/io_dir.c
  src/io_cgi.c
  src/io_cgi.c
  src/compat_unix.c
)

target_link_libraries(shttpd
  libdl.so
  libpthread.so
  libpthread.so
  libcrypto.so
  libssl.so
)
