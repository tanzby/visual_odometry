# 相机模型

相机模型的相关定义参考自：

* 《视觉SLAM十四讲》



## 单目模型

假设图像已经进行了去畸变处理，考虑世界坐标系下一点 $\boldsymbol{p}^w=[p_x,p_y,p_z]^T$，通过相机观测得到，该相机相对于世界坐标系的位姿由$\mathbf T_{cw} \in \mathbf{SE}(3)$ 描述，此时该点的相机坐标为
$$
\boldsymbol{p}^c = \left(\mathbf T_{cw}[\boldsymbol{p}^w,1]^T\right)_{0:2} = [p_x^c,p_y^c,p_z^c]^T
$$
将$\boldsymbol{p}_c$投影到归一化平面$z=1$上，得到归一化相机坐标
$$
\boldsymbol{p}^{c,z=1} = 
\begin{bmatrix}p_x^c/p_z^c \\ p_y^c/p_z^c \\1
\end{bmatrix}
$$
使用内参数矩阵$\mathbf{K}$，对应到像素坐标
$$
\begin{aligned}
\boldsymbol{p}^{uv} &= \mathbf{K} \boldsymbol{p}^{c,z=1}=[u, v,1]^T \\\\
\mathbf{K} &= 
\begin{bmatrix}
f_x & 0 & c_x \\ 
0 & f_y & c_y \\
0 & 0   &  1
\end{bmatrix}
\end{aligned}
$$


## 双目模型

假设两个相机的光圈都位于$x$轴上，它们的距离为双目相机的基线（baseline），记为$b$，考虑相机坐标系下（一般以左相机为基准，但两相机坐标系下深度都一致，无需特别区分）的空间点$\boldsymbol{p}^c=[p_x^c,p_y^c,p_z^c]^T$，由三角形相似可得该点的深度
$$
p_z^c = \frac{fb}{d},\quad d = u_L-u_R 
$$
其中$d$表示左右图横坐标（$u$坐标）之差，称为时差（Disparity）

