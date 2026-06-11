![alt text](image.png)
fault表挪到图中这个路径
![alt text](image-1.png)
双击工具.exe文件会打开工具
![alt text](image-2.png)
选择e009_e007项目（用离线扫表的话项目可以随便选，选了之后会自动选择离线fault表路径）
![alt text](image-3.png)
生成的fault表文件转移到圈内dsar-hq仓库替换掉相应车型的diag_lcfg.c文件，其中生成的.h文件的两个宏替换掉仓库diag_lcfg.h对应宏即可。