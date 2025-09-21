/*
 * @Author: Elaina
 * @Date: 2024-10-17 19:03:01
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-10-17 19:43:30
 * @FilePath: \MDK-ARM\Lib\Lib_Common.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef __LIB_COMMON_H
#define __LIB_COMMON_H
#include <new>
#include "FreeRTOS.h"
extern "C"
{
    void Error_Handler();
}
void* operator new(std::size_t size) noexcept(false); // ✅ C++11及以上

void operator delete(void *pointer) throw();
void delay(uint32_t ms);
template <typename T>
class CustomAllocator
{
public:
    using value_type = T;
    CustomAllocator() = default;
    template <typename U>
    CustomAllocator(const CustomAllocator<U> &) noexcept {}
    T *allocate(std::size_t n)
    {

        if (n > std::size_t(-1) / sizeof(T))
        {
            Error_Handler();
        }
        if (auto p = (T *)(pvPortMalloc(n * sizeof(T))))
        {
            return p;
        }
        else
        {
            Error_Handler();
        }
    }
    void deallocate(T *p, std::size_t)
    {
        vPortFree(p);
    }
};
//列表实现
template <typename T>
class LibList_t
{
public:
    template <typename Func>
    void Foreach(Func func)
    {
        for (int i = 0; i < size; i++)
        {
            func(data[i]); // 调用传入的函数对象
        }
    }
    void Foreach(void (*func)(T &))
    {
        for (int i = 0; i < size; i++)
        {
            func(data[i]);
        }
    }
    T &Find(bool (*func)(T &))
    {
        for (int i = 0; i < size; i++)
        {
            if (func(data[i]))
            {
                return data[i];
            }
        }
        return nullptr;
    }
    void Add(T &&item)
    {
        if (size == 0)
        {
            data = new T[1];
            data[0] = item;
            front_ptr = data;
            tail_ptr = data + 1;
            size = 1;
        }
        else
        {
            T *temp = new T[size + 1];
            for (int i = 0; i < size; i++)
            {
                temp[i] = data[i];
            }
            temp[size] = item;
            delete[] data;
            data = temp;
            size++;
            tail_ptr = data + size;
        }
    }
    T &operator[](int index)
    {
        return data[index];
    }
    T *data;

private:
    T *front_ptr;
    T *tail_ptr;
    int size;
};
class Promise_t
{
public:
};
/*只有返回的Promise*/
class SimpleStatus_t
{
public:
    SimpleStatus_t() = default;
    bool isResolved()
    {
        return _isResolved;
    }
    void resolve()
    {
        _isResolved = true;
    }
    void start()
    {
        _isResolved = false;
    }
private:
    bool _isResolved = false;
};
#endif