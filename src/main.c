#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "time.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "sdkconfig.h"

// ***************************************************************************
// Config section
// ***************************************************************************

// ***** Hardware 
#define HCSR04_TRIGGER GPIO_NUM_27
#define HCSR04_ECHO GPIO_NUM_35

#define LED GPIO_NUM_26

// ***** Behavior
#define N_SAMPLES 3 // average of given number of measurements will be used
#define SAMPLE_PERIOD_IN_SECS 2

// ***** Globals
const double sonicspeed = 340.0 * 100.0 / 1000.0 / 1000.0; // sonic speed in cm per usec

// ***************************************************************************
// Helper functions
// ***************************************************************************

void panic ( char *comment, char *file, int line ) {
    printf("PANIC: %s in file <%s> at line %d>\n",comment,file,line);
}

#define PANIC(x) panic(x,__FILE__,__LINE__);

double absolute ( double v ) {
    return v<0 ? -v : v;
}

// ***************************************************************************
// Dealing with time issues
// ***************************************************************************
double time_interval ( struct timeval *start, struct timeval *stop) {
    double usecs = ((double) (stop->tv_sec - start->tv_sec)) * 1e6;
    usecs = usecs + ((double) (stop->tv_usec - start->tv_usec));
    return usecs;
}

void delay_count (int c) {
    volatile int vc = c;
    while (vc>0) vc--;
}

void sleep_ms ( int milliseconds ) {
    vTaskDelay(milliseconds / portTICK_PERIOD_MS);
}

// ***************************************************************************
// Helper functions - understanding the ESP32 device
// ***************************************************************************
void print_chip_info () {
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is a ESP32 chip with %d CPU cores, WiFi%s%s, ",
        chip_info.cores,
        (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
        (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    printf("silicon revision %d, ", chip_info.revision);
    printf("%dMB %s flash\n", spi_flash_get_chip_size() / 1024 / 1024,
        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    // printf("portTICK_PERIOD_MS == %d\n",portTICK_PERIOD_MS);
    printf("Version of the ESP-IDF framework: %s\n",esp_get_idf_version());
    printf("FreeRTOS version is %s\n",tskKERNEL_VERSION_NUMBER);
    printf("configMAX_PRIORITIES=%d\n",configMAX_PRIORITIES);
    // printf("configCPU_CLOCK_HZ=%d\n",configCPU_CLOCK_HZ);
    printf("configTICK_RATE_HZ=%d\n",configTICK_RATE_HZ);
    printf("configMINIMAL_STACK_SIZE=%d\n",configMINIMAL_STACK_SIZE);
    // printf("configTOTAL_HEAP_SIZE=%ld\n",configTOTAL_HEAP_SIZE);
    printf("Free heap size is %d\n",esp_get_free_heap_size());
    fflush(stdout);
}

void show_value ( gpio_num_t led, int active, int v ) {
    // printf("Show value: ");
    bool leading_blank = true;
    for (int i=15; i>=0; i--) {
        bool digit = (v >> i) & 0x1;
        if (leading_blank & !digit) continue; else leading_blank = false;
        // if (digit) printf("1"); else printf("0");
        gpio_set_level(led, active);
        sleep_ms(digit ? 400 : 100);
        gpio_set_level(led, 1-active);
        sleep_ms(200);
    }
    // printf("\n");
}

// ***************************************************************************
// check_distance - returns loop count that correlates with echo length
// ***************************************************************************
int64_t check_distance ( gpio_num_t trigger, gpio_num_t echo ) {
    static int max_count = 100000;
    gpio_set_level(trigger,1);
    ets_delay_us(100);
    gpio_set_level(trigger,0);
    int count_0 = 0;
    while ((gpio_get_level(echo) == 0) && (count_0<max_count))
        count_0++;
    if (count_0 == max_count)
        return -1;
    // printf("count_0 == %d\n",count_0);
    int64_t start = esp_timer_get_time();
    int count_1 = 0;
    while ((gpio_get_level(echo) == 1) && (count_1<max_count))
        count_1++;
    int64_t stop = esp_timer_get_time();
    if (count_1 == max_count)
        return -1;
    // printf("count_1 == %d\n",count_1);
    return stop-start;
}

// ***************************************************************************
// task_check_distance
// ***************************************************************************
void task_check_distance ( void *params ) {
    double last_distance = 0.0;
    struct timeval now;
    while (true) {
        gettimeofday(&now,NULL);
        time_t seconds_passed = now.tv_sec;
        int samples = 0;
        double echo_usecs = 0.0;
        for (int m=0; m<N_SAMPLES; m++) {
            int64_t usecs = check_distance(HCSR04_TRIGGER,HCSR04_ECHO);
            if (usecs > 0) {
                echo_usecs += ((double) usecs);
                samples += 1;
            }
            sleep_ms(100);
        }
        if (samples == 0)
            printf("%10ld: No object detectable\n",seconds_passed);
        else {
            echo_usecs /= ((double) samples);
            double distance = (echo_usecs * sonicspeed) / 2.0;
            printf("%10ld: object at distance %f cm\n",seconds_passed,distance);
            double change = absolute(last_distance - distance);
            if (change > 1.0) {
                printf("----------: Distance change > 10mm: %f at time %d\n",distance,(int) now.tv_sec);
            }
            last_distance = distance;
            show_value(LED,1,(int) distance);
        }
        vTaskDelay(SAMPLE_PERIOD_IN_SECS * 1000 / portTICK_PERIOD_MS);
    }
}

// ***************************************************************************
// MAIN
// ***************************************************************************

void app_main()
{
    // Immediate I/O configuration
    // HC-SR04
    gpio_pad_select_gpio(HCSR04_TRIGGER);
    gpio_set_direction(HCSR04_TRIGGER, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(HCSR04_ECHO);
    gpio_set_direction(HCSR04_ECHO, GPIO_MODE_INPUT);

    // RGB LED
    gpio_pad_select_gpio(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    
    // Wait 2 secs for console to connect
    sleep_ms(2000);
    printf("Cistern Water Level ...!\n");
    print_chip_info();  
    printf("sonic speed is %f cm/usec\n",sonicspeed);

    xTaskCreate(&task_check_distance,"Task_Check_Distance",2048,NULL,5,NULL);

    while(1) {
        sleep_ms(5000);
    }
}