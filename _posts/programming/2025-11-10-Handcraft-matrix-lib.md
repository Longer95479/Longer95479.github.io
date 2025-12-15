---
layout: post
title:  "Handcraft Your Own Matrix Library"
date:   2025-11-10 22:30:00 +0800
tags: 
  - programming
  - math 
categories:
  - programming
  - math 
---

视觉惯性里程计的前后端的一些计算，涉及矩阵的运算，一些问题的求解，最终都会转变为方程组的求解，
也与矩阵的运算或线性代数相关，想要从零实现一个 VIO 的前端，主要涉及到 *多视角几何（MVS）*，
而 MVS 中的问题的求解绝大部分也都是转换为方程组的求解，因此从零实现一个矩阵运算库就变成了必要条件。

本文将记录笔者用 C++ 实现一个简单的矩阵运算库（类似于 Eigen 库）的过程。

代码仓库：[handcraft-MVS](https://github.com/Longer95479/handcraft-MVS)。

实现的功能有：

- 矩阵多种方式的构造（初始化）与析构

- 矩阵的操作或运算
    - 加减乘，以及标量除法
    - 赋值
    - 矩阵的转置、L2 范数
    - 矩阵取块

- 矩阵的分解
    - QR分解（householder 方法）
    - SVD分解（迭代使用 QR 分解）

- 方程组的求解（QR分解 ＋ 回代）

其中，比较值得展开的内容有：

- 类内数据结构的选择（主要服务于取块操作 `.block<>()`，而 `.block<>()` 主要服务于后续的 QR 分解实现），以及 `.block<>()` 的实现方式
  - `T**` 为核心数据结构，即二维数组指针，用来访问二维数组
  - `.block<blk_rows, blk_cols>(start_row, start_col)` 函数是取块操作，从较大矩阵中返回连续的一部分，实现方式是增加 5 个成员 `bool is_block_` `int block_rows` `int block_cols` `int start_row` `int start_col` ，以及一个特殊的构造函数，该构造函数将接受 `.block<>()` 的模板参数和函数参数，构造一个新的矩阵，构造过程将会对上述成员变量赋值。

- 拷贝构造函数和移动构造函数的不同实现
  - 拷贝构造函数主要是重新申请一片内存，然后将数据复制进来
  - 移动构造函数则是转换所有权，将自己的指针指向已有的内存区域，然后将右值对象的对应指针指向 `nullptr`

- 析构函数的实现应避免内存的重复释放和泄露
  - 防止重复释放：因为 `.block<>()` 返回的是块矩阵，并不拥有自己的内存，所以在析构这类对象的时候不应当释放内存，通过增加对 `is_block_` 的判断来实现
  - 防止泄露：因为指针是 `T**` 类型，所以在释放的时候应先释放一系列 `T*` 的内存，再释放 `T**` 本身指向的内存

- 赋值操作符重载，对左值引用和右值引用类型的输入做不同的实现，即 *拷贝* 或 *移动* 的实现
  - 保证大小相同
  - 拷贝：内存的重写
  - 看二者的取块属性，如果一致，则是所有权转移，否则进行拷贝
  - **NOTE**：对于所有权转移，一定要首先释放旧内存，再指向新内存，最后将右值的指针指向 `nullptr`，如果忘却第一步，则会出现内存泄漏，随着程序的运行每次调用移动拷贝都会带来内存的增长，直至程序被系统杀掉。

- QR 分解中，通过模板函数的递归实现有限次数的循环

  ```c++
  void func() 
  {
      for (int i = 0; i < N; i++) {
        do_something(i);
      }
  }
  ```

  ```c++
  template<int i>
  void func()
  {
      do_something(i);
  
      if constexpr (i == N)
          return;
      else
          func<i + 1>();
  
  }
  ```

- 迭代使用 QR 分解来实现 SVD 分解

  ```c++
  U = I, V = I
  for (int i = 0; i < N; i++) {
  
    A = Ui R
    RT = Vi CiT
  
    A <- Ci
    U <- U * Ui
    V <= V * Vi
  }
  return {U, C, V}
  ```

- QR 分解和回代实现方程组的求解

- 在模板函数中，当你在依赖类型（该类型依赖于模板参数）的对象上访问成员模板时，必须显式使用 template 关键字。
  - C++ 编译器对于模板代码，会进行 two-phase lookup。
    1. Phase 1，第一步检查，只检查模板代码是否有语法错误，但涉及到和模板类型参数相关的部分会跳过。检查的范围包括是否有明显的语法错误比如用了不存在的关键字、少了分号等，其中也会检查那些和模板类型参数无关的函数、类型、方法是否已经被声明，这和编译器检查普通代码的流程很相似
    2. Phase 2, 这一步会往模板的参数里带入实际的类型，编译器会重新推导整个模板代码在当前的类型下是否合法
    3. 两步骤是为了更快速地将类型参数不相关的问题排除，这样在保证模板代码语法正确性的同时尽量保证了泛型代码的灵活性，理想中也能让模板的编写者更快发现问题而不是把问题延迟到类型推导之后。
  - 对函数参数 A：编译器“知道这是一个对象”，能延后解析。即 `如果是依赖表达式（dependent expression），编译器可以延后解析，在实例化阶段再确认。`
  - 对局部变量 H_tmp：编译器“只知道它是依赖类型”，但此时必须立刻解析语法树。一旦出现 <，语法二义性必须立即消除 → 你必须写 template

  ```c++
  template<typename T, int rows, int cols>
  class Matrix {
  public:
      template<int i>
      std::pair<Matrix<T, rows, rows>, Matrix<T, rows, cols>>
      calQR(const Matrix<T, rows, rows>& H, const Matrix<T, rows, cols>& A)
      {
          Matrix<T, rows, rows> H_tmp;   // 局部变量
          H_tmp.template block<rows - i, rows - i>(i, i);
          A.block<rows - i, 1>(i, i);
      }
  };
  ```


TODO:

- [ ] 赋值操作的深拷贝浅拷贝实现（避免析构时内存重复释放）

- [ ] 目前的 QR 分解、SVD分解和方程组求解只支持 *矩阵行数大于列数* 的情况，待扩展
