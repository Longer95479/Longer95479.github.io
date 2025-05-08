---
layout: post
title:  "Back to the Basics! Essentials of Modern C++ Style"
date:   2025-05-08 23:00:00 +0800
tags: 
  - programming
categories:
  - programming
---


Back to the Basics

- Tour to C++

## 1 prefer range-for 

why do this: 
```
for ( auto i = begin(c); i != end(c); i++ ) { ... use(*i); ...}
```

when you can do this: 
```
for (auto& e: c) { ... use(e); ... }
```
## 2 Use smart pointers effectively, but still ** use lots of raw * and & **, they're great!

Dont's use owning `*`, `new` or `delete`. Except: Encapsulated inside the implementation of low-level data structures.

For "new", use `make_unique` by default, make_shared if it will be shared. For "delete", write nothing.

Example: 
- c++98 (now mostly wrong)
```c++
widget* factory();
void caller()
{
  widget* w = factory();
  gadget* g = new gadget();
  use( *w, *g );
  delete g;
  delete w;
}
```
- Modern C++:
```c++
unique_ptr<widget> factory();
void caller()
{
  auto w = factory();
  auto g = make_unique<gadget>();
  use( *w, *g );
}
```

NB: Non-Owing */& Are Still Great.

Why: 被调用者在调用者的生命周期内，不需要做所有权转移（you don't need ownership transfer down to call stack unless you're going to take something out of the call stack）

Example:
- C++98 "Classic":
```c++
void f( widget& w )  // if required
{
  use(w);
}

void g( widget* w )  // if optional
{
  if(w) use(*w);
}
``` 
- Modern C++ "Still Classic":
```c++
void f( widget& w )  // if required
{
  use(w);
}

void g( widget* w )  // if optional
{
  if(w) use(*w);
}
``` 
How to use those functions:
```c++
auto upw = make_unique<widget>();
...
f( *upw );

auto spw = make_shared<widget>();
...
g( spw.get() );
```

Antipatterns Hurt Pain Pain

Antipattern #1: Parameters (Note: Any refcounted pointer type.)
```
void f( refcnt_ptr<widget>& w )
{
  use(*w);
} // ?

void f( refcnt_ptr<widget> w )
{
  use(*w);
} // ?!?!
```
前者想避免计数，但会带来歧义：要更改指针的指向吗

后者带来性能上的恶化：每次进出函数都会递增和递减，而这些操作是原子操作，需要同步，因此开销并不小。

在这二种进行选择是一种 过早悲观：因为在都很复杂的操作中选择了更快，或者说在更快和更复杂中进行选择。
因此不如直接使用 raw */&

Antipattern #2: Loops (Note: Any refcounted pointer type.)
```
refcnt_ptr<widget> w = ...;
for (auto& e: baz) {
  auto w2 = w; // ?!?!?!?!
  use( w2, *w2, w, *w, whatever);
} // ?!?!?!?!
```
在循环中复制智能指针，缺点仍是带来大的开销

example:
In late 2013, Facebook RocksDB changed from pass-by-value shared_ptr to pass */&. QPS improveed 4x (100K to 400K) in one benchmark. 
[http://tinyurl.com/gotw91-example](http://tinyurl.com/gotw91-example)

什么时候才 copy/assigns smart pointer?

- refcounted smart pointers are about managing the owned object's lifetime.

- Don’t pass a smart pointer as a function parameter unless you want to use or manipulate the smart pointer itself, such as to share or transfer ownership.

- Prefer passing objects by value, *, or &, not by smart pointer.

- Express a “sink” function using a by-value unique_ptr parameter.

- Use a non-const unique_ptr& parameter only to modify the unique_ptr.

- Don’t use a const unique_ptr& as a parameter; use widget* instead.

- Express that a function will store and share ownership of a heap object using a by-value shared_ptr parameter.

- Use a non-const shared_ptr& parameter only to modify the shared_ptr. Use a const shared_ptr& as a parameter only if you’re not sure whether or not you’ll take a copy and share ownership; otherwise use widget* instead (or if not nullable, a widget&).


```c++
unique_ptr<widget> factory();		// source - produces widget
void sink( unique_ptr<widget> );	// sink - consumes widget
void reseat( unique_ptr<widget>& );	// "will" or "might" reseat ptr
void thinko( const unique_ptr<widget>& )// usually not what you want

shared_ptr<widget> fractory();		// source + shared ownership
	// when you know it will be shared, perhapsby factory itself
void share( shared_ptr<widget> )	// share - "will" make a copy 
					// and retain refcount
void reseat( shared_ptr<widget>& )	// "will" or "might" reseat ptr
void may_share( const shared_ptr<widget>& ) // "might" retain refcount
					// conditionally keep a copy
```

- [ ] 疑惑：返回类型是 unique_ptr<widget> ，可以 return widget* 类型吗

Not quite done: One guiline missing, and it applies to any RC pointer type, in almost any language / library

Guideline: Dereference Unaliased+Local RC Ptrs
只对 非别名且局部 的 引用计数指针 解引用 
否则指针可能在使用前已经被完全释放了 [0:27:19]



## 3 use auto

- To make type track, deduce:
```
auto var = init;
```
- To make type stick, commit:
```
auto var = type{ init };
// or
type var{ init };
```
Reason:
- Counterarguments reflects bias to code against implementation, not interfaces.
- conrrectness + maintainability 当类型变动时，能够自动推导类型,减少思考，避免修改遗漏带来的编译错误 或 隐式转换（额外开销 or narrow conversion 精度下降，引出了第三点理由）
- performance
- usability (一些变量名字很长，很难拼写，所以用 auto 可以少打字)

Left-toright auto style:
[0:44:26]

(The) case where you can't use "auto style"
[0:47:35]

增加可读性的一个例子：
[0:49:18] 显式地表达了转换
```c++
base* pb = new derived();  // old fasion
unique_ptr<base> pb = make_unique<derived>();  // modern but too subtle
auto pb = unique_ptr<base>{ make_unique<derived>() }; //explicit and clear
```

## 4 Use return-by-value way more often
but Don't overuse pass-by-value

Just as exception safey isn't about writing try and catch, using move semantick isn't all about writing move and &&

table:[0:58:29]

When do I write rvalue &&? Only to write rvalues.

no alloc -> noexcept

an example/question to show when use rvalue optim:

```
class employee {
  std::string name_;
public:
  void set_name(/* ?? */) { /* ?? */ }
}
```
1. default: `void set_name( const std::string& name ) { name_ = name; }`

1 copy assignment in body, for small string (SSO, small string optimization) no alloc, for large string <50% time alloc

2. optimized to steal from rvalues: add overload for string&& + move
```
void set_name( std::string&& name ) noexcept { 
  name_ = std::move(name); }
```
- pass a named obj: 1 copy assignment, <50% alloc, as before
- pass a temporary: 1 move assignment, ~5 ints, no alloc -> noexcept
- Note: conbinatorial if multiple "in + retain copy" parameters (e.g. 3 params need to write 2^3 = 8 function overload)

3. string + move, optimized to steal from rvalues, without overloading: 
```
void set_name( std::string name ) noexcept {
  name_ = std::move(name);
}
```
- pass named obj, 1 copy construction (100% alloc if long) + move op=
- pass a temporary: 1 move construction, 1 move assignment(~5 ints, no alloc -> noexcept-ish)
- this "noexcept" is problematic, why? because it is technically true, your code will complie, but you push the operation that might throw into the caller. If one usr has an lvlaue and he calls this function, a copy will be performed, that copy could perform an allocation, that allocation could fail, that could throw.


for vector & large string
| | construction | assignment / operator= |
|-|-|-|
| default | $$ |x|
| move | $$ | $ |
| copy | $$$ | $$$ |


## 5 One Quiz

```
void foo( X&& x );

template<class Y>
void foo( Y&& y );
```

`X&&` 和 `Y&&` 有什么区别呢？
- `X&&` 表示函数只接受 rvalue
- `Y&&` 表示函数接受任何类型，包括 lvalue、rvalue、const、nonconst等，称之为 forward reference


## 6 Dessert Slides

这个正好解决了我这段时间的疑问。

Use *turple* for multiple return values.

```
// C++98

pair<set<string::iterator, bool>> result = myset.insert( "Hello" );
if (result.second) do_something_with( result.first );  // workaround

// C++11 - sweet backword compat
auto result = myset.insert( "Hello" );  // nicer syntax, and the
if (result.second) do_something_with( result.first );  // workaround still work

// C++ - sweet forword compat, can treat as multiple return values
tie( iter, sucess ) = myset.insert( "Hello" );
if (success) do_somthing_with( iter );

```

## Summery
loops
pointers & references
smart pointers
variable declaaration
parameter passing

---

`explicit` 关键字用于一个参数的函数，一般用于 构造函数，禁止隐式转换的发生


boost::bind 可以在运行时动态创建 可调用对象

如果函数不在类内，则 
```
int add(int a, int b) { return a + b; }
auto add2 = boost::bind(add, 2, _1);
add2(3); // 5
```
否则：
```
one_class(){
  ...
  _one_thread = std::thread(std::bind(&one_class::method, this));
  ...
}
```



