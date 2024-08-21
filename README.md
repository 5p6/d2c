### 1.introduction
This is a method for calibrating and aligning RGBD cameras.

### 2.Environment
* ubuntu 22.04
* GNU Tools 11.x
* OpenCV 4.x
* Eigen 3.x
* Boost

### 3.Example
calib  --> align 
#### 3.1 build
To satisfy the environment,run command 
```bash
sudo apt install  libopencv-dev \
                  libeigen3-dev \
                  libboost-dev \
                  build-essential \
                  cmake \
                  git
```
adn then
```bash
git clone 
```
If the environment are linux,you need to change some varable in the `CMakeLists` ,which include the fmt and opencv path,the 
```shell
cd project_path
```
and
```shell
bash build.sh
```

So you could make use of the executable file to calib
#### 3.2 calibration
Run using the command line,for rgb images calibration
```bash
./build/rgbd_calib --rgb-dir /mnt/hgfs/share/dataset/calib/rgbd/calib/calib/rgb/ --depth-dir /mnt/hgfs/share/dataset/calib/rgbd/calib/calib/infrared/ -h 6 -w 7 -o . -s 20
```
this program will calculate the parameters,which needed by align program,those parameters will saved in `align.yaml`,like
```yaml
%YAML:1.0
---
rgbK: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 1.0330670202081678e+03, 0., 9.3046861250618576e+02, 0.,
       1.0310455772701362e+03, 5.3662586300152179e+02, 0., 0., 1. ]
depthK: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 3.6013258447643722e+02, 0., 2.6048421452847896e+02, 0.,
       3.5982078122427777e+02, 2.1293927876427088e+02, 0., 0., 1. ]
R: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 9.9976239492351493e-01, -7.7578583427777136e-03,
       2.0370796025952997e-02, 7.8386493541675473e-03,
       9.9996171422686042e-01, -3.8891717345808990e-03,
       -2.0339844470890121e-02, 4.0479271747460499e-03,
       9.9978492937855823e-01 ]
t: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [ 4.0526717379990224e+01, 3.5935658368717966e+00,
       -4.0260805147425174e+00 ]
```

you could use the `--help` option to check the help context,like
```shell
./build/rgbd_calib --help
```

#### 3.3 align
Finally, the alignmentcommand is as follows
```bash
./build/rgbd_align -r /mnt/hgfs/share/dataset/calib/rgbd/calib/test/kehu/rgb.png -d /mnt/hgfs/share/dataset/calib/rgbd/calib/test/kehu/depth.png -p ./align.yaml
```
align by inputting images, calibration parameters, and extrinsic data.This program will show the rgb image and aligned depth image.

### Appendix
The goal of alignmentis to convert $(u_{ir},v_{ir})$ to $(u_{rgb},v_{rgb})$,as

$$\begin{align}
  H(u_{ir},v_{ir}) = (u_{rgb},v_{rgb})
\end{align}$$

so we need to find H to satisfy the equation as (1).the specific principle of RGBD is introduced, and its mathematical formula is as follows.The projection equations for RGB cameras and depth cameras are as follows,rgb : 

$$\begin{align}
  \begin{bmatrix}
    u_{rgb} \\ v_{rgb} \\ 1
  \end{bmatrix}  = \frac{1}{Z_{rgb}}K_{rgb}[R_{rgb}|t_{rgb}] \begin{bmatrix}
    X_w\\ Y_w \\ Z_w \\ 1
  \end{bmatrix}
\end{align}$$

ir : 

$$\begin{align}
  \begin{bmatrix}
    u_{ir} \\ v_{ir} \\ 1
  \end{bmatrix}  = \frac{1}{Z_{ir}}K_{ir}[R_{ir}|t_{ir}] \begin{bmatrix}
    X_w\\ Y_w \\ Z_w \\ 1
  \end{bmatrix}
\end{align}$$

The mathematical formula above allows for simultaneous equations,Assuming $M_{rgb},M_{ir}$ is a three-dimensional point in the coordinate system of relative camera RGB and depth

$$
\begin{align}
\begin{cases}
  M_{rgb} =  [R_{rgb}|t_{rgb}] \begin{bmatrix}
    X_w\\ Y_w \\ Z_w \\ 1
  \end{bmatrix} = R_{rgb}M_w + t_{rgb}\\
  M_{ir} = [R_{ir}|t_{ir}] \begin{bmatrix}
    X_w\\ Y_w \\ Z_w \\ 1
  \end{bmatrix} = R_{ir}M_w + t_{ir}
\end{cases}
\end{align}
$$

which

$$\begin{align}
M_w = \begin{bmatrix}
    X_w\\ Y_w \\ Z_w 
    \end{bmatrix},M_{rgb} = \begin{bmatrix}
    X_{rgb}\\ Y_{rgb} \\ Z_{rgb} 
    \end{bmatrix},M_{ir} = \begin{bmatrix}
    X_{ir}\\ Y_{ir} \\ Z_{ir} 
    \end{bmatrix}
\end{align}$$

We can obtain the following equation from equation (3)

$$\begin{align}
  M_{ir} = R_{ir}R_{rgb}^TM_{rgb} + t_{ir} - R_{ir}R_{rgb}^Tt_{rgb}
\end{align}$$

Convert the factors in the above equation into the following parameters

$$
\begin{align}\begin{cases}
  R = R_{ir}R_{rgb}^T \\
  t = t_{ir} - R_{ir}R_{rgb}^Tt_{rgb}
\end{cases}
\end{align}
$$

Or convert it into the following equation

$$\begin{align}
\begin{cases}
  M_{rgb} = RM_{ir} + t \\
  R = R_{rgb}R_{ir}^T \\
  t = t_{rgb} - R_{rgb}^TR_{ir}t_{ir}
\end{cases}
\end{align}$$

then we could get $M_{ir}$ like

$$\begin{align}
  M_{ir} = \begin{bmatrix}
    \frac{d (u_{ir} - c_x^{ir})}{f_x^{ir}} \\ \frac{d(v_{ir} - c_y^{ir})}{f_y^{ir}} \\ d
  \end{bmatrix}
\end{align}$$

and convert the $M_{ir}$ to $u_{rgb},v_{rgb}$,

$$\begin{align}
  u_{rgb} =  \frac{f_x^{rgb}}{Z_{rgb}} X_{rgb} + c_x^{rgb} \\
  v_{rgb} = \frac{f_y^{rgb}}{Z_{rgb}}Y_{rgb} + c_y^{rgb}
\end{align}$$

so we could convert  $(u_{ir},v_{ir})$ to $(u_{rgb},v_{rgb})$.To conclude,we 
* calibrate the rgb and ir camera,get data $K_{rgb,}R_{rgb},t_{rgb},K_{ir},R_{ir},t_{ir}$
* calculate the  $R$ and $t$

$$
\begin{align}
\begin{cases}
  R = R_{rgb}R_{ir}^T \\
  t = t_{rgb} - R_{rgb}R_{ir}^Tt_{ir}
\end{cases}
\end{align}
$$

* calculate the Point $M_{ir} = (X_{ir},Y_{ir},Z_{ir})$

$$
\begin{align}
\begin{cases}
  X_{ir} = \frac{d (u_{ir} - c_x^{ir})}{f_x^{ir}} \\ 
  Y_{ir} = \frac{d(v_{ir} - c_y^{ir})}{f_y^{ir}} \\ 
  Z_{ir} = d
\end{cases}\rightarrow M_{ir} \end{align} $$

* calculate the Point $M_{rgb} = (X_{rgb},Y_{rgb},Z_{rgb})$
 
$$\begin{align}
  M_{rgb} = RM_{ir} + t
\end{align}$$
* Finally, calculate the coordinates $(u_{rgb},v_{rgb})$

$$\begin{align}
  u_{rgb} =  \frac{f_x^{rgb}}{Z_{rgb}} X_{rgb} + c_x^{rgb} \\
  v_{rgb} = \frac{f_y^{rgb}}{Z_{rgb}}Y_{rgb} + c_y^{rgb}
\end{align}$$

remeber that $Z_{rgb} \approx d$ .
