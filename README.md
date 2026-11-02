# 基于 PGL50H FPGA 的实时图像畸变矫正与动态仿射变换系统

> 全国大学生嵌入式芯片与系统设计竞赛 · 紫光同创 FPGA 赛道相关作品  
> **PGL50H · OV5640 · DDR3 · RGB565 · HDMI 1080P@60Hz · Python 上位机**

![FPGA](https://img.shields.io/badge/FPGA-PGL50H-blue)
![Tool](https://img.shields.io/badge/IDE-PDS%202025.2-green)
![Language](https://img.shields.io/badge/Language-Verilog%20%7C%20Python-orange)

本项目面向机器视觉中镜头畸变、姿态变化及复杂光照等问题，基于**紫光同创 PGL50H FPGA**构建完整的硬件视频处理通路：OV5640 图像采集、DDR3 三帧缓存、定点几何映射、双线性插值、动态仿射变换、图像增强与 HDMI 输出。PC 仅负责通过 UART 下发控制命令及参数，**不参与任何图像像素的实时运算**。

主处理通路针对 **1920×1080** 图像设计，HDMI 采用 **1920×1080@60Hz** 显示时序；**60Hz 是输出刷新率，不代表摄像头以 60fps 采集不同画面**。

## 1. 系统架构

```mermaid
flowchart TD
    A[OV5640 1920×1080] --> B[RGB565 采集 / Gamma 增强]
    B --> C[DDR3 三帧缓存]
    C --> D[32 行循环缓存]
    D --> E[定点径向/切向畸变映射]
    E --> F[RGB565 双线性插值]
    C --> G[独立缩略帧缓存]
    G --> H[旋转 / 二维平移]
    H --> I[缩略图双线性插值]
    F --> J[显示模式选择]
    I --> J
    J --> K[透明 OSD / 硬件参考线]
    K --> L[HDMI 1080P@60Hz]
    M[Python PC 上位机] -->|UART 115200 8N1| N[参数校验及状态控制]
    N -.-> E
    N -.-> H
    N -.-> K
