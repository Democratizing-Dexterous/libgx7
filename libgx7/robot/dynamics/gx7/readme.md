## 动力学辨识

### Regressor

`calc_dynamics.py`里面有一个类`CalcDynamics`，可以计算出GX7的动力学回归矩阵，即：

$$
\tau = H(q, \dot{q}, \ddot{q})\beta
$$

其中，$\tau$是关节力矩，$H$是回归矩阵，$\beta$是需要辨识的参数。

运行`calc_dynamics.py`，回输出回归矩阵的维度:

```
regressor shape: (7, 69)
```

### OLS

把辨识的结果保存在`betas/xxx.npy`里面