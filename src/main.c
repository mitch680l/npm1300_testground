#include <zephyr/kernel.h>

int main(void)
{
        return 0;

        while(1) {
                k_sleep(K_MSEC(1000));
        }
}
