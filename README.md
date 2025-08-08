# libgx7

GX7-Arm

## USB2CAN权限设置

```
sudo cp 99-myusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```
