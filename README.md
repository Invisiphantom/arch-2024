# Arch-2024-Spring-Fudan

> 2024 Spring Computer Organization and Architecture(H) Course Code Repository.

Arch-2024-Sping-FDU  
│── build：仿真测试时才会生成的目录  
│── difftest：仿真测试框架  
│── ready-to-run：仿真测试文件目录（包括汇编文件和二进制文件等）  
│── verilate：verilator部分仿真文件目录  
│── vsrc：需要写的CPU代码所在目录  
│　　├── include：头文件目录  
│　　├── src：你将在这个目录下完成CPU的代码编写  
│　　　　　└── core.sv：CPU核主体代码  
│　　├── util：访存接口相关目录  
│　　└── SimTop.sv  
│── Makefile：仿真测试的命令汇总  
└── README.md: 此文件  


## 环境配置

https://github.com/chipsalliance/verible

```bash
git clone https://github.com/verilator/verilator.git
cd verilator && git checkout v4.210
sudo apt install -y perl make autoconf g++ flex bison ccache gtkwave libsdl2-dev zip
autoupdate
autoconf
./configure
make -j12
sudo make install
verilator --version

sudo apt install universal-ctags
wget https://github.com/chipsalliance/verible/releases/download/v0.0-3671-gf2731544/verible-v0.0-3671-gf2731544-linux-static-x86_64.tar.gz
tar -xvf  verible-v0.0-3671-gf2731544-linux-static-x86_64.tar.gz
sudo mv verible-v0.0-3644-g6882622d /usr/local/verible
```
