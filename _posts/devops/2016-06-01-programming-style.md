---
layout: article
title:  "《编程格调》笔记"
tags: CodeStyle
key: devops-programming-style
---

# 表达

* 简单、直接、清晰
* 使用库函数
* 避免使用临时变量
* 让机器干脏活
* 用函数调用替代重复的表达式
* 加括号避免歧义
* 选择不会被混淆的变量名
* 用“电话测试”检查可读性

# 控制结构

* 使用数组避免重复的控制流
* 选择可以简化程序的数据表示方法

# 程序结构

* 模块化，使用子例程
* 不要修补烂代码，重写它
* 对于递归定义的数据结构使用递归过程

# 输入输出

* 校验输入的合理性和合法性
* 保证输入数据不会违背程序的限制
* 利用文件结束符或结束标志来终止输入，不要让用户计数
* 使用统一的输入格式

# 常见错误

* 确保所有变量在使用之前都被初始化
* 注意对不等式进行正确的分支
* 避免循环有多个出口
* 在边界值上测试程序
* 小心“差一”错误
* 预防性编程
* 10.0乘以0.1不等于1.0
  - 不要用浮点数做累计
* 不要比较浮点数是否相等

# 效率和测试工具

* 先做对再做快
* 让编译器进行平凡优化
* 不要勉强的复用代码，应进行改编
* 保正特殊情况真的有特殊性
* 在程序中放置测试语句

# 文档

* 确保注释和代码一致
* 不要过度注释
* 用缩进体现程序的逻辑结构
* 记录你的数据规划