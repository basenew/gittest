升级流程:
1,拷贝文件:upgrade.upg,DS10240C-APP-V1.1-20210106.bin到ftp://10.20.18.10 中去
2,登录到:http://10.20.18.10/upgrade.html,页面,即可进行升级:增量升级,强制升级,单独升级

注意:若有新版本bin文件则需要更改upgrade.upg文件对应的内容,如下:
	version_year:2021
	version_date:0106
	bin:DS10240C-APP-V1.1-20210106.bin

	version_year,bin后面数字前4位
	version_date,bin后面数字后4位
	bin,要升级的bin文件名
	更改后保存,用升级流程的顺序进行升级