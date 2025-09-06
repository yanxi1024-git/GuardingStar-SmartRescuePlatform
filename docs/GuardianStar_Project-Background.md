# 守护星 · 智慧救援平台 / Guardian Star · Intelligent Rescue Platform

## 一、设计背景 / Design Background
地震、洪水等灾害常导致道路受阻，救援人员难以进入，物资运输存在风险。为探索智能装备在救援中的应用，我们设计了“守护星 · 智慧救援平台”。该平台以智能无人小车为核心，模拟灾害场景下的应急运输，具备遥控、巡线、避障和视频监控功能，并在架构上预留了无人直升机扩展接口，形成“车机协同”的救援雏形，贴近真实应用需求。  

Disasters such as earthquakes and floods often block roads, making it difficult for rescuers to enter and increasing the risks of material transportation. To explore the application of intelligent equipment in rescue missions, we designed the “Guardian Star · Intelligent Rescue Platform”. This platform, centered on an intelligent unmanned vehicle, simulates emergency transportation in disaster scenarios. It features remote control, line-tracking, obstacle avoidance, and video monitoring, while reserving an expansion interface for unmanned helicopters. This forms an initial prototype of “vehicle-aircraft collaboration” that closely meets real application needs.  

## 二、设计思路 / Design Concept
系统采用“远程遥控 + 自动巡线”的双模式：在复杂或空旷地形下，通过手机 APP 遥控并结合 ESP 无线视频监控模组实现实时画面回传；在隧道或特殊路线中，灰度传感器驱动小车自动巡线，超声波模块负责避障。板载模式切换按钮支持一键切换运行模式，RGB 灯提示状态。未来版本中，顶部无人直升机可通过舵机保护舱起降，扩展车机协同救援。  

The system adopts a dual-mode design of “remote control + automatic line-tracking”: in complex or open terrains, the vehicle can be remotely controlled via a mobile app, with real-time video feedback enabled by the ESP wireless video module; in tunnels or special routes, a grayscale sensor enables automatic line-tracking, while an ultrasonic module handles obstacle avoidance. An onboard mode-switching button allows one-click switching between modes, with an RGB light indicating status. In future versions, a drone can take off and land from a servo-controlled protection bay, extending vehicle-aircraft collaborative rescue.  

## 三、制作过程 / Production Process
1) **硬件选型和搭建 / Hardware selection and construction**  
平台基于 Arduino UNO 搭配扩展板，集成电机驱动、麦克纳姆轮底盘、灰度与超声波传感器、RGB 状态灯、舵机、电压检测及 ESP 视频模组。  
The platform is based on Arduino UNO with expansion boards, integrating motor drivers, a mecanum wheel chassis, grayscale and ultrasonic sensors, RGB status lights, servos, voltage detection, and an ESP video module.  

2) **软件设计和优化 / Software design and optimization**  
系统是在相对成熟稳定的开源框架基础上进行二次开发，重点优化了模式切换的互斥逻辑、巡线与遥控的协调控制、电压采样与滤波的可靠性，以及串口通信协议的解析效率。整体采用 C/C++ 实现，并引入非阻塞流程设计，使运动控制、监控与检测任务能够并行运行，提升了系统稳定性。  
The system is developed on relatively mature and stable open-source frameworks, with key optimizations in mode-switching logic, coordination between line-tracking and remote control, reliability of voltage sampling and filtering, and efficiency of serial communication protocol parsing. Implemented mainly in C/C++, the system adopts a non-blocking process design, enabling motion control, monitoring, and detection tasks to run in parallel, thereby improving system stability.  

3) **AI 协同制作 / AI-assisted development**  
在整个项目过程中，结合和借助大模型 AI 进行任务分解、代码优化和方案验证，形成“Ai + 创意制作”的新模式，有效提高了开发效率与实现质量。  
Throughout the project, large-model AI was used for task decomposition, code optimization, and solution verification, forming a new model of “AI + creative making”, which effectively improved development efficiency and implementation quality.  

## 四、使用方法 / Usage Instructions
智能小车启动后，默认进入远程遥控模式，用户可通过手机 APP 控制小车运动，实时查看视频画面与电池状态。按下板载模式切换按钮，可进入自动巡线模式，小车沿黑色轨迹自主运行，并在遇到障碍物时自动调整或停止；再次按下按钮可返回遥控模式。RGB 灯提示运行状态，舵机可驱动机械装置或未来的无人机停机坪。通过灵活的模式切换与扩展功能，智能小车能模拟灾害中的应急运输场景。  

After startup, the smart vehicle defaults to remote control mode, where users can control its movement via a mobile app while viewing real-time video and battery status. Pressing the onboard mode-switching button enables automatic line-tracking mode, where the vehicle follows a black track autonomously and adjusts or stops when encountering obstacles. Pressing the button again returns it to remote control mode. The RGB light indicates operating status, while the servo can drive mechanical devices or a future drone landing pad. Through flexible mode switching and expansion features, the smart vehicle can simulate emergency transportation scenarios during disasters.  
