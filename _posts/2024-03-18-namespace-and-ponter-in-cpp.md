---
layout: post
title:  "Namespace and ponter in cpp"
date:   2024-03-18 18:33:00 +0800
tags: 
  - c++
categories:
---

Use pointer to creat and acess nested class or struct instance.

```c++
#include <iostream>
#include <vector>
#include <memory>

class A
{
public:
    typedef std::shared_ptr<A> Ptr;
    
    struct ASub
    {
    public:
        typedef std::shared_ptr<ASub> Ptr;
        int x_;
        
        ASub(int x);
    };
    
    int x_;
    Ptr a_ptr_;
    ASub::Ptr a_sub_ptr_;
    
    A(int x);
};

A::A(int x):x_(x){}
A::ASub::ASub(int x):x_(x){}

int main()
{
    A::Ptr a_ptr(new A(95479));
    std::cout << a_ptr->x_ << std::endl;
    a_ptr->a_sub_ptr_.reset(new A::ASub(23));
    std::cout << a_ptr->a_sub_ptr_->x_ << std::endl;
    
}
```
Output:
```bash
95479
23
```
---
cpp online compiler: [Coliru online compiler](https://coliru.stacked-crooked.com/)
