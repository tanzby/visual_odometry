# 李代数



李代数的相关实现（$\mathbf{SE}(3)$、$\mathbf{SO}(3)$）参考自：

1. 《视觉SLAM十四讲》

2. [https://github.com/jbehley/SuMa](https://github.com/jbehley/SuMa)

3. [Local accuracy and global consistency for efficient visual SLAM](https://core.ac.uk/download/pdf/9833208.pdf)

4. [A tutorial on SE(3) transformation parameterizations and on-manifold optimization](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.468.5407&rep=rep1&type=pdf)

   

## 基本定义

描述在$\mathbb{R}^{3}$上的刚体变化的变换矩阵构成**特殊欧式群**（special Euclidean group），用 $\bf{SE}(3)$表示，其元素的组成为$4\times4$矩阵，有如下的结构：
$$
\bf{T} =
\left(\begin{array}{c|c}{\mathbf{R}} & {\mathbf{t}} \\ \hline \mathbf{0 _ { 1 \times 3 }} & {1}\end{array}\right)
$$
其中$\mathbf {R}\in \mathbf{SO}(3), \mathbf{t} = [t_x, t_y, t_z]^T \in \mathbb{R}^{3}$

通常，使用$[\cdot]_{\times}$ 定义一个向量的反对称矩阵(skew-symmetric matrix)，有
$$
\begin{bmatrix}x\\y\\z\end{bmatrix}_{\times}=\begin{bmatrix}0 & -z & y \\z &  0 & -x \\-y & x & 0 \\\end{bmatrix}_{\times}
$$
其相反操作使用$[\cdot]_{\triangledown}$表示 ，有
$$
\left[\begin{bmatrix} 0 & -z & y \\z &  0 & -x \\-y & x & 0 \\\end{bmatrix}\right]_{\triangledown}=\begin{bmatrix}x\\y\\z\end{bmatrix}
$$

## $\mathbf{SO}(3)$ 上的李代数

对于$\mathbf{SO}(3)$，其指数映射（Exponential map）表示：
$$
\begin{aligned} \exp : \mathfrak{s o}(3) & \mapsto \mathbf{SO}(3) \\ \boldsymbol{\omega} & \mapsto \mathbf{R}_{3 \times 3} \end{aligned}
$$
其闭式表达为:
$$
e^{\boldsymbol{\omega}} \equiv e^{[{\boldsymbol{\omega}}]_{\times}}=\mathbf{I}_{3}+\frac{\sin \theta}{\theta}{[{\boldsymbol{\omega}}]_{\times}}+\frac{1-\cos \theta}{\theta^{2}}{[{\boldsymbol{\omega}}]_{\times}}^{2}
$$
该对应关系是满射的，其中，$\theta=\left\|\boldsymbol{\omega}\right\|_2$ ，$[{\boldsymbol{\omega}}]_{\times}$ 为$\boldsymbol{\omega}\in \mathbb{R}^{3}$对应的反对称矩阵.



对于$\mathbf{SO}(3)$，其对数映射由下式给出：
$$
\begin{aligned} \ln : \mathbf{S O}(3) & \mapsto \mathfrak{s o}(3) \\ \mathbf{R}_{3 \times 3} & \mapsto \boldsymbol{\omega} \end{aligned}
$$
该对应关系同时也是满射的，是对数映射的逆运算，其闭式表达为：
$$
\begin{aligned}
\cos \theta &=\frac{\operatorname{tr}(\mathbf{R})-1}{2} \\
\ln (\mathbf{R}) &=\frac{\theta}{2 \sin \theta}\left(\mathbf{R}-\mathbf{R}^{\top}\right)\\\\
\boldsymbol{\omega} &=[\ln (\mathbf{R})]_{\triangledown} 
\end{aligned}
$$


## $\mathbf{SE}(3)$ 上的李代数

令
$$
\mathbf{v}=\left(\begin{array}{c}{\mathbf{t}} \\ {\boldsymbol\omega}\end{array}\right)
$$
表示$\mathfrak{s e}(3)$ 上的$\mathbb R^6$上的向量，由两个独立的$\mathbb R^3$上的向量组成，其中$\mathbf{t}$表示位移，$\boldsymbol w$表示旋转。并令
$$
\mathbf{A}(\mathbf{v})=\left(\begin{array}{cc}{[\boldsymbol{\omega}]_\times} & {\mathbf{t}} \\ {0} & {0}\end{array}\right)
$$
表示将$\mathbb R^6$上的$\mathbf{v}$变换为矩阵的形式。



对于$\mathbf{SE}(3)$，其指数映射表示：
$$
\exp : \mathfrak{s e}(3) \mapsto \mathbf{SE}(3)
$$
该对应关系是满射，其闭式表达为:
$$
\begin{aligned} 

e^{\mathbf{v}} \equiv e^{\mathbf{A}(\mathbf{v})}=\left(\begin{array}{cc}{e^{[\boldsymbol\omega]_{\times}}} & {\mathbf{V} \mathbf{t}} \\ {0} & {1}\end{array}\right) 

\end{aligned}
$$
其中
$$
\mathbf{V}=\mathbf{I}_{3}+\frac{1-\cos \theta}{\theta^{2}}[\boldsymbol{\omega}]_{\times}+\frac{\theta-\sin \theta}{\theta^{3}}[\boldsymbol{\omega}]_{\times}^{2}
$$
对于$\mathbf{SE}(3)$，其对数映射表示：
$$
\begin{aligned} \ln : \mathbf{S E}(3) & \mapsto \mathfrak{s e}(3) \\ \mathbf{A}(\mathbf{v}) & \mapsto \mathbf{v} \end{aligned}
$$
这一映射是完备的，其闭式表达为：
$$
\begin{aligned}

\mathbf{v} &=
\begin{pmatrix}
    \mathbf{t}'\\
  \hline
    \boldsymbol{w}
\end{pmatrix} = 
\begin{pmatrix}
    x'\\
    y'\\
    z'\\
  \hline
    w_1\\
    w_2\\
    w_3\\
  \end{pmatrix}
\\

\boldsymbol{\omega} &=[\ln \mathbf{R}]_{\triangledown} \\ 

\mathbf{t}^{\prime} &=\mathbf{V}^{-1} \mathbf{t}

\end{aligned}
$$