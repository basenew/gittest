# SW 开关量检测

# 使用说明

用户需要关注两个文件，```sw.h```和```sw_port.c```

1、```sw.h```中需要定义：
```
SW_POLL_SLICE         轮询线程的轮询周期
SW_THREAD_STACK_SIZE  轮询线程的栈大小
SW_THREAD_PRIORITY    轮询线程的优先级
```
填入开关量的枚举编号：
```
typedef enum
{
    SW0 = 0,
    SW1,


    SW_NUM
} sw_idx_t;
```
2、```sw_port.c```中需要定义：

各开关量的默认参数：
```
static sw_unit_t swtable[] = 
{
{SW0, 31, SW_POLL_SLICE, 2*SW_POLL_SLICE, 0, 0, 0, io_init, io_read, RT_NULL,},
{SW1, 32, SW_POLL_SLICE, 2*SW_POLL_SLICE, 0, 0, 0, io_init, io_read, RT_NULL,},
};
```
各开关量的初始化```Init()```以及读取状态```Read()```接口:
比如，对于IO检测类型，可以定义为：
```
static int32_t io_init(sw_unit_t* unit)
{
    rt_pin_mode(unit->Pin, PIN_MODE_INPUT_PULLUP);
    return 0;
}

static uint8_t io_read(sw_unit_t* unit)
{
    return rt_pin_read(unit->Pin);
}
```


# 接口说明

## 初始化

```
int32_t sw_init(void)
```
初始化成功后，系统会创建一个独立的线程用于轮询各个开关量。

## 使能某个开关量检测

```
void sw_unit_enable(sw_idx_t idx,  uint32_t ms,  uint32_t dbms)

@param idx : 枚举编号
@param ms  : 检测周期
@param dbms: 消抖时间，至少2倍ms，且为ms整数倍
```

## 失能某个开关量检测

```
void sw_disable(sw_idx_t idx)

@param idx : 枚举编号
```

## 绑定某个开关量的触发回调

```
void sw_attach(sw_idx_t idx,  sw_callback_t cb)

@param idx : 枚举编号
@param cb  : 回调函数
```

## 解绑某个开关量的触发回调

```
void sw_detach(sw_idx_t idx)

@param idx : 枚举编号
```



# 注意事项


# 遗留问题


