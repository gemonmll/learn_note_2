### 01 Autosar概述
> Autosar概述
> 一套文档，一个框架 一个方法论
> ![alt text](image.png)
> 环境搭建
> ![alt text](image-1.png)
> eb和mcal 集成到vector中 重要
> ![alt text](image-2.png)
> 最小系统的搭建
> ![alt text](image-3.png)
### 02 最小工程介绍
> ![alt text](image-4.png)
> 最小系统需要的模块
 1）基础需求功能 vlinkgen 链接脚本初始化
mcu不一定需要使用，可以自己写初始化代码
> ![alt text](image-5.png)
> #### 2.1 OS详解
> os介绍
> os存在 sc1 sc2 sc3等级
> 1) 使用OS的原因(优先级抢占、现场保护恢复、访问资源控制、共享资源访问)
由OSEK OS发展而来，额外增加schedule table、多核、时间保护、内存保护
> ![alt text](image-7.png)
> os application (tasks ISRs Alarms CounterS)集合 类似用户组
> 同一个OS Application可以互相访问，不同需要授权
> 一个核中可以有多个，一般来说配两个就行（systemApplication,用户使用的OsApplication）
> app内访问使用RTE，app间访问用IOC(inter-osApp-comm),ecu间访问使用Com
> ![alt text](image-8.png)
> ### 2.2 task (bcc ecc)
> bcc 与 ecc, 多了wait状态
> 注意bcc是terminateTask
> ecc是waitEvent(while 1 循环)
> ![alt text](image-9.png)
> ![alt text](image-10.png)
> bcc1和ecc1的区别
> bcc1是不可抢占任务，但ecc1是取决于配置
> ![alt text](image-11.png)
> ### 2.3 tick Counter Alarm
> systemTimer 配置计算
> 软件中断和硬件中断
> ![alt text](image-12.png)
> 一个counter 可以关联多个Alarm
> 但一个alarm只会触发一个task
> 激活task,setEvent,调用callback
> ![alt text](image-13.png)
> scheTable可以看作是Alarm的升级版
> ### 2.4 event
> 必须依托Extend task,只有所属task才能清除和等待task
> ![alt text](image-14.png)
> ### 2.5 interrupt
一类中断不受os管控，不执行虚线部分，直接执行中断处理函数 
二类中断受os管控（如下图所示），在处理中断前会进行执行一些os的代码（会触发一次调度等等）
timer中断应该是二类优先级最高的中断、否则bsw可能会出现一个os limit错误
> ![alt text](image-16.png)
> 最小系统只用到了system tick 中断
> 二类中断中优先级最高的
> ![alt text](image-17.png)
> ### 2.6 resource
> 天花板优先级
> 用于线程同步
> ![alt text](image-18.png)

## 3 bswM和EcuM介绍
> ### bswM介绍
对通信进行控制
ecu状态进行处理(最小系统配置重点)
初始化设置（一些外围接口不在这里初始化
bswm流程图 （startup -> wakeup -> run(正常运行) -> postrun -> shutdown）,保持在run状态下需要保持条件
ecu状态理解、理解概念 action list \  rules
> ![alt text](image-19.png)
> ECU状态流程图
> ![alt text](image-20.png)
> davinci下的配置
> rules就是判断条件
> ![alt text](image-22.png)
> ![alt text](image-23.png)
> ![alt text](image-24.png)
> action 代表真正要做的事
> ![alt text](image-25.png)
> action list  各个状态中的action 组合
> ![alt text](image-26.png)
> 示例： run to postRUN 状态解析
> ![alt text](image-27.png)
> 条件成立或是不成立下的action
> ![alt text](image-28.png)
> ### EcuM 介绍
> 大部分工作被BSWM做了，EcuM配置较少
> 有两种模式 fixed(较老，固定的上下电流程) 和 flexible（通常选择，通过配置进行管理）
> ![alt text](image-29.png)
> ![alt text](image-30.png)
> EcuM上下电状态，重要，后续也会介绍
> 选择走sleep状态还是shutdown状态，或是reset
> 走到sleep状态也会选择 go poll (代码运行)或是 go halt（代码停）
> ![alt text](image-31.png)
> ![alt text](image-32.png)
> ![alt text](image-33.png)

## 4 EcuC RTE App介绍
>  ### ECUC介绍
> 主要工作，初始化，对hardware的一些设置，pdur（最小系统中没有）
> ![alt text](image-34.png)
> ecuc  硬件配置 多核配置
> ![alt text](image-35.png)
> 
> ### RTE
> 中转站，对上链接swc，对下对接通信栈，存储栈，OS，CDD
> ![alt text](image-36.png)
> ### application 介绍
> 各个模块间的调用关系图
> ![alt text](image-41.png)
> develop工具 （app components types）
> ![alt text](image-38.png)
> service components 相关的swc
> data type 做了单位转化
> ![alt text](image-39.png)
> app port 和 service port
> ![alt text](image-40.png)