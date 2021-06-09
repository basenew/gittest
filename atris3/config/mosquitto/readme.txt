证书生成步骤：
1、openssl genrsa -out ca.key 2048
2、openssl req -new -x509 -days 73000 -key ca.key -out ca.crt -subj "/C=CN/ST=BJ/L=BJ/O=HD/OU=dev/CN=10.20.18.2/emailAddress=atris@ubtrobot.com"
3、openssl genrsa -out server.key 2048
4、openssl req -new -key server.key -out server.csr -subj "/C=CN/ST=GuangDong/L=ShenZhen/O=UBTech/OU=ToB/CN=10.20.18.2/emailAddress=atris@ubtrobot.com"
5、openssl x509 -req -days 73000 -in server.csr -CA ca.crt -CAkey ca.key -CAcreateserial -out server.crt