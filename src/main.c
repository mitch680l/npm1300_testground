#include <zephyr/kernel.h>
#include "config.h"
#include "gnss.h"
#include <zephyr/debug/thread_analyzer.h>


LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);


int main(void)
{
        gnss_int();

        // Print thread stats after initialization to see peak stack usage
        k_sleep(K_MSEC(100));
        printk("\n=== Thread Analysis After Init ===\n");
        thread_analyzer_print(0);

        int loop_count = 0;
        while(1) {
            gnss_main_loop();
            k_sleep(K_SECONDS(1));
            printk("%s\n", json_payload);

            // Print thread stats every 60 seconds to monitor ongoing usage
            loop_count++;
            if (loop_count % 60 == 0) {
                printk("\n=== Thread Analysis (runtime) ===\n");
                thread_analyzer_print(0);
            }
        }
    return 0;
}
