#include "Lib_Common.h"
#include "task.h"
void delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}
void* operator new(std::size_t size) noexcept(false)
{
    return pvPortMalloc(size);
}

void operator delete(void *pointer) throw()
{
    vPortFree(pointer);
}