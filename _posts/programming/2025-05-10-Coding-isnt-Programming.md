---
layout: post
title:  "Coding isn't Programming"
date:   2025-05-10 16:00:00 +0800
tags: 
  - programming
categories:
  - programming
---

[Coding isn't Programming(Leslie Lamport)](https://www.bilibili.com/video/BV1HJRyYgE4j/?spm_id_from=333.1387.favlist.content.click&vd_source=e371652571b1539bbd501fb7adb6cfc4)

算法不应该被用特定的编程语言来表示，
应该关注想法，而不是使用哪种语言。
这是一种简化，因为具体的编程语言需要考虑 *高效的执行* 和 *大型程序的组织*

考虑一种比较难的场景，编写并发（concurrancy）的算法和程序。并发算法和程序都很难写，但算法更简单。

所以，我们需要抽象（abstraction），合理的抽象可以帮助我们把注意力放在真正重要的地方上，减少思考的负担，
但这是在有思考的前提下。引用 Lamport 的话：

> 1. thinking before you code
> 2. thinking at a higher level than code

> For most programs, you should write two things:
> 1. What the program does.
> 2. How the program does it.

算法适用性更广，在此处我们将这个更高层次的思考但比较局限的思考称之为 抽象（abstraction）

> 我们应当花时间去思考如何简化 What，这将节省后续编码的时间。

以 *求数组最大值* 为例:

描述1：从一个数组中找出最大值

无法应对数组为空的情况，思考另一种表达方式。

描述2：从一个数组中找出最大值，且当数组为空时将输出设置为 error

描述3：如何找出满足 >= multiset 中所有元素的最小值


接下来我们给出了 How：

```shell 
given A = {{2, 3, 2}}

initial B <- A, m <- -inf

while (B isn't empty):
  i <- anyone ele of B
  remove i in B
  if (i > m): m <- i
```

我们如何保证不会出现一个反例 A 使得输出的结果不正确呢？
这时候便要思考 执行（execution）。

什么是执行？常见的回答：执行是一系列步骤（step），步骤是一部分代码的执行。
通常更好的回答：

> 执行是一系列状态序列，步骤是相邻的状态转换对，描述了某一部分代码的执行

> 状态是一系列的量，其下一步的值只由当前值决定，其他量视为输入

可以这样用 `状态` 来抽象的原因是：
- 程序（programs）在计算机上执行
- 计算机下一步执行的内容，只和当前的状态 和 输入 有关，不依赖于之前的状态
- 我们可以借此概念，来帮助我们证明这个程序的正确性，而无需考虑具体的编程语言


对于这个问题

```shell 
given A = {{2, 3, 2}}

initial B <- A, m <- -inf

while (B isn't empty):
  i <- anyone ele of B
  remove i in B
  if (i > m): m <- i
```


还是抽象或简化的思想，我们只关注最核心的部分：
- 抽离出初始化
- 将 while 的判断 和 while 内的语句当成一个步骤（step）

我们列出状态是如何变化的：
```python
[B = {{2, 3, 2}}, m = -inf]
  | step 1
  v
[B = {{3, 2}}, m = 2]
  | step 2
  v
[B = {{2}}, m = 3]
  | step 3
  v
[B = {{}}, m = 3]
  | step 4
  v
[B = {{}}, m = 3]
 stop
```


为什么这个抽象的程序是正确的呢？为什么所有的执行，都在 状态X 等于正确答案时停止呢?

- 在执行的任意时刻，下一个状态仅取决于当前状态

- 所以，在执行的任意时刻，下一状态 等于 能让程序终止的状态，也仅取决于当前状态  

- X 只有在程序等于正确的值时终止，这必须依赖于 对每个状态都为真的某个条件

- 对每个状态都为真的某个条件，称之为 程序/算法的不变量


我们无法理解一个算法为什么是对的，除非我们找到它的不变量，并能从不变量推导出正确性。

> You don't understand why a program does the right thing such as terminate with correct answer, unless you know the invariant that ensures it does the right thing.

为了证明一个条件是不变量，需要让这个条件满足：

1. 对 初始状态 为真
2. 若对 任意状态 为真，那么该条件对 下一状态 也为真 

为了证明程序是正确的：
3. 证明 终止条件 + 不变量 -> 正确性

以本文的例子来说明：

- 程序正确意味着：程序终止的时候应当满足 m == max(A) 

- 不变量：max( {{ m, max(B) }} ) = max(A)
  1. 对 初始条件 为真：m = -inf, B = A, max( {{ -inf, max(A) }} ) = max(A)
  2. 对本状态为真，则对下一状态也为真：

  3. 当终止时，B = {{}}，max( m, max({{}}) ) = m = max(A)，所以不变量配合终止条件，可以推导出正确性，即 程序中止时 m = max(A)

条件1 和 条件3 是平凡的，关键在于 条件2 的证明。

```
max( \{\{ m1, max(B1) \}\} ) = max(A)

B2 = B1 without i

if i <= m1, then m2 = m1 
-> max( \{\{ m2, max(B2) \}\} ) = max( \{\{ m1, max(B1 without i) \}\} )
if i is max(B1):  = max( \{\{ m1, i \}\} ) = m1 = max( \{\{ m1, B1 \}\} )
if i isn't max(B1): = max( \{\{ m1, max(B1) \}\} )

if i > m1, m2 = i
-> max( \{\{ m2, max(B2) \}\} ) = max( \{\{ i, max(B1 without i) \}\} ) 
if i is max(B1): = i = max( \{\{ m1, max(B1)\}\} )
if i isn't max(B1): = max(B1) = max( \{\{ m1, max(B1) \}\} )

so
max( \{\{ m2, max(B2)\}\} ) = max( \{\{ m1, max(B1)\}\} ) = max(A)
```



我们应该学习如何证明这个条件是正确的

> Because understanding why something is invariant is very important.
>I think you should learn how, because I except those who can't to be among the first programmers replaced by AI.

在具体实现上，可以用 最小整数 或 错误值 来表示 $-\infty$

以上，我们证明了当程序终止时，能够给出正确答案。但并没有证明程序一定会终止。实际上当 A 的元素有无穷多时，程序永远不会终止。



