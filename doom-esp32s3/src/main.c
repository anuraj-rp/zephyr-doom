#include <stdio.h>
#include <zephyr/kernel.h>

int main(void)
{
    printf("Hello World! %s\n", CONFIG_BOARD);
}
