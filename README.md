# esp32-ws2812-tcp
基于esp32，可通过小程序进行控制的ws2812驱动

配网方法：关注“安信可科技”微信公众号，使用菜单中的微信配网功能

状态指示灯：红色接25脚 绿色接26脚 蓝色接27脚

​	蓝色闪烁：正在连接wifi

​	蓝绿色闪烁：等待智能配网

​	绿色长亮：连接成功，正在进行灯带检测，完成后熄灭

​	红色长亮：wifi连接断开，重连成功后红灯熄灭



ws2812接16脚

扩展继电器接15脚 继电器默认低电平触发 通电后默认触发