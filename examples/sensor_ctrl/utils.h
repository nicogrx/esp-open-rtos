#ifndef UTILS_H
#define UTILS_H
#define delay_ms(ms) vTaskDelay((ms) / portTICK_PERIOD_MS)
#define systime_ms() (xTaskGetTickCount() * portTICK_PERIOD_MS)
#endif
