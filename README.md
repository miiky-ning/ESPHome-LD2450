# ESPHome-LD2450
Hi-Link LD2450 For ESPHome

用于海凌科LD2450雷达感应模块连接ESPHome的驱动和ESPHome代码。
复制'components'文件夹和其中的驱动文件到Home Assistant的config/esphome目录下。
Copy the folder 'components' to home assistant '/config/esphome'.

changelog:
  1.支持设置区域过滤设置（指定区域坐标内不扫描，暂时只支持一个区域）
  2.支持区域内人体数量的测量
  3.添加“恢复出厂设置”功能
  4.添加“重启模块”功能
