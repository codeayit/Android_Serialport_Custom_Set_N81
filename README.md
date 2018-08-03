# Android_Serialport_Custom_Set_N81
1.此 安默++ Android 串口调试工具（原名 安默） 串口调试工具 支持 数据位 校验位 校验方式的设定(默认是N-8-1)

2.项目完善了 https://github.com/silencefun/ComTest 其中的 Android_SetN81 对原作者表示敬意。

# 完善部分

1.将原 serial_port.c 修改为 serial_port.cpp （.c 暂无法编译出相应abi的.so ）

2.完善了奇偶校验功能（此处已经请教过原作者，我也只是站在巨人的肩膀上，招了招手）

3.将native 代码 恢复到 android_serialport_api（google原创） 包下面

# 结语

1.此前项目代码都是第三方公司提供的 .so 包 仅仅支持 arm-v7a 

2.本人开发和公司都受制于人，因此沉下心来攻克此技术 

3.欢迎 star or fork
