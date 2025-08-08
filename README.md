# libgx7文档

## USB2CAN权限设置
执行如下代码设置USB2CAN权限，然后重新拔插USB或者重启：
```
sudo cp hardware/99-myusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 安装依赖
```
pip install -r requirements.txt
```
注意事项：
* `pinocchio`的安装为：
```
pip install pin
```
