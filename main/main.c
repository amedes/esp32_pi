/* PI - Machin's formula

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <malloc.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <driver/gpio.h>

#include "emp.h"

#define TAG "pi"

#ifdef CONFIG_ENABLE_LED
#define BLINK_LED 1
#endif

#ifdef BLINK_LED
#define LED_GPIO CONFIG_LED_PIN
#endif


struct FACTOR {
    int coeff;
    int dividend;
    int divisor;
};

static const struct FORMULA {
    char *name;
    struct FACTOR fc[5];
} formula[] = {
    { "Machin", { { 4, 1, 5 }, { -1, 1, 239 } } },
    { "Klingenstierna", { { 8, 1, 10 }, { -1, 1, 239 }, { -4, 1, 515 } } },
    { "Euler", { { 5, 1, 7 }, { 2, 3, 79 } } },
    { "Euler(2)", { { 4, 1, 5 }, { -1, 1, 70 }, { 1, 1, 99 } } },
    { "Gauss", { { 12, 1, 18 }, { 8, 1, 57 }, { -5, 1, 239 } } },
    { "Stormer", { { 6, 1, 8 }, { 2, 1, 57 }, { 1, 1, 239 } } },
    { "Stormer(2)", { { 44, 1, 57 }, { 7, 1, 239 }, { -12, 1, 682 }, { 24, 1, 12943 } } },
    { "Takano", { { 12, 1, 49 }, { 32, 1, 57 }, { -5, 1, 239 }, { 12, 1, 110443 } } },
    { NULL },
};

struct SEND_ITEM {
    void (*func)(emp_t *dst, emp_t *src, int operand);
    emp_t *dst;
    emp_t *src;
    int operand;
};

struct RECEIVE_ITEM {
    int ret;
};

static uint32_t work_time = 0;
static uint32_t work_count = 0;
static uint32_t work_wait = 0;

static QueueHandle_t s_queue, r_queue;

#define SENDQ_LEN 32
#define RECEIVE_LEN 32

static void work_task(void *p)
{
    struct SEND_ITEM sitem, *s = &sitem;
    struct RECEIVE_ITEM ritem = {
        .ret = 0,
    };
    uint32_t t0 = 0;

    while (1) {
#ifdef BLINK_LED
       gpio_set_level(LED_GPIO, 0);
#endif
       if (xQueueReceive(s_queue, s, portMAX_DELAY) != pdTRUE) {
	   ESP_LOGW(TAG, "xQueueReeive: fail");
	   continue;
       }
#ifdef BLINK_LED
       gpio_set_level(LED_GPIO, 1);
#endif

       if (s->func) {
           t0 = esp_timer_get_time();
           (*s->func)(s->dst, s->src, s->operand);
           work_time += esp_timer_get_time() - t0;
	   work_count++;
       } else {
	   xQueueSend(r_queue, &ritem, portMAX_DELAY);
       }
    }
}

static inline void send_command(QueueHandle_t queue, void (*func)(), emp_t *dst, emp_t *src, int operand)
{
    struct SEND_ITEM item = {
	.func = func,
	.dst = dst,
	.src = src,
	.operand = operand,
    };

    if (xQueueSend(queue, &item, portMAX_DELAY) != pdTRUE) {
	ESP_LOGE(TAG, "xQueueSend fail");
	exit(1);
    }
}

static void div_x2(emp_t *dst, emp_t *src, int divisor, int dividend)
{
    //send_command(s_queue, mp_copy, dst, src, 0);

    if (dividend > 1) { // for Euler's formula
	send_command(s_queue, emp_mul, dst, src, dividend * dividend);
    }

    if (divisor == 8) {
	send_command(s_queue, emp_srl, dst, src, 6);
    } else if (divisor < 256) {
	send_command(s_queue, emp_div, dst, src, divisor * divisor);
    } else if (divisor < 65536) {
	send_command(s_queue, emp_div, dst, src, divisor);
	send_command(s_queue, emp_div, dst, dst, divisor);
    } else if (divisor == 110443) { // 110443 = 179 * 617
	send_command(s_queue, emp_div, dst, src, 179 * 179);
	send_command(s_queue, emp_div, dst, dst, 617);
	send_command(s_queue, emp_div, dst, dst, 617);
    } else {
	ESP_LOGE(TAG, "divisor too large: %d", divisor);
	exit(1);
    }
    send_command(s_queue, NULL, NULL, NULL, 0); // end of command
}

static int arctan(emp_t *pi, int coeff, int divisor, int dividend)
{
    emp_t term, work, work2;
    uint8_t addsub;
    uint16_t value;
    struct RECEIVE_ITEM item;

    emp_init(&term, pi->size);
    emp_init(&work, pi->size);
    emp_init(&work2, pi->size);

    if (coeff < 0) {
	value = -coeff;
	addsub = 0;
    } else {
	value = coeff;
	addsub = 1;
    }
    
    emp_setint(&work, value * dividend);

    if (divisor == 8) {
	emp_srl(&work, &work, 3);
    } else if (divisor < 65536) {
        emp_div(&work, &work, divisor); // 1/x
    } else if (divisor == 110443) { // 110443 = 179 * 617
        emp_div(&work, &work, 179); // 1/x
        emp_div(&work, &work, 617);
    } else {
	ESP_LOGE(TAG, "divisor too large: %d", divisor);
	exit(1);
    }
    if (addsub) {
        emp_add(pi, &work); // first factor
    } else {
        emp_sub(pi, &work); // first factor
    }
    addsub = !addsub;

    // term x^3
	if (dividend > 1) { // for Euler's formula
	    emp_mul(&work, &work, dividend * dividend);
	}

        if (divisor == 8) {
	    emp_srl(&work, &work, 6);
	} else if (divisor < 256) {
    	    emp_div(&work, &work, divisor * divisor); // work /= x^2
	} else if (divisor < 65536) {
    	    emp_div(&work, &work, divisor); // work /= x
    	    emp_div(&work, &work, divisor); // work /= x
	} else if (divisor == 110443) { // 110443 = 179 * 617
    	    emp_div(&work, &work, 179 * 179); // work /= x^2
    	    emp_div(&work, &work, 617);
    	    emp_div(&work, &work, 617);
	} else {
	    ESP_LOGE(TAG, "divisor too large: %d", divisor);
	    exit(1);
	}

    int m = (pi->size - 1) * 32 * log(2) / (2 * log((float)divisor / dividend)) + 1;

    emp_t *w0 = &work, *w1 = &work2, *wt;

    for (int i = 1; i < m; i++) {
	int n = 2 * i + 1;

	if (n > UINT16_MAX) {
	    ESP_LOGE(TAG, "dvisor overflow error: %d\n", n);
	    break;
	    exit(1);
	}

	if (i > 1) {
	    // wait multiplying by x^2 task
            uint32_t t = esp_timer_get_time();
	    if (xQueueReceive(r_queue, &item, portMAX_DELAY) != pdTRUE) {
	    	ESP_LOGE(TAG, "xQueueReceive fail");
	    	exit(1);
	    }
	    work_wait += esp_timer_get_time() - t;
	    //printf("work_task time: %d us\n", item.ret);
	    // swap w0, w1
	    wt = w0;
	    w0 = w1;
	    w1 = wt;
	}

	if (i + 1 < m) div_x2(w1, w0, divisor, dividend);

   	emp_div(&term, w0, n); // term = work / (2i+1)

	if (addsub) {
	    emp_add(pi, &term);
	} else {
	    emp_sub(pi, &term);
	}
	addsub = !addsub;
    }

    emp_deinit(&work2);
    emp_deinit(&work);
    emp_deinit(&term);

    return m; // number of factors
}

#define MEGA	(1000 * 1000)

static void calc_pi(const struct FORMULA *fo)
{
    emp_t pi;
    uint32_t t0, t1;
    int f[4];
    int m;
    const struct FACTOR *fp;
    int n;
    int sum;

    if (!fo->name) return;

    printf("\ncalculate Pi using %s's formula\n", fo->name);

    fp = fo->fc;
    printf("pi/4 = ");
    while (fp->coeff != 0) {
	printf("%+d*arctan(%d/%d)", fp->coeff, fp->dividend, fp->divisor);
	fp++;
    }
    printf("\n");
    
    m = (int)(65536 * log(fo->fc->divisor / fo->fc->dividend) / (32 * log(2)) + 1) & ~1;

    emp_init(&pi, m);

    t0 = esp_timer_get_time();

    fp = fo->fc;
    n = 0;
    while (fp->coeff != 0) {
        f[n++] = arctan(&pi, fp->coeff * 4, fp->divisor, fp->dividend);
	fp++;
    }

    t1 = esp_timer_get_time();

    printf("pi = ");
    emp_printdec(&pi);

    sum = 0;
    printf("factors: ");
    for (int i = 0; i < n; i++) {
	printf((i == 0) ? "%d" : " + %d", f[i]);
	sum += f[i];
    }
    printf(" = %d\n", sum);

    t1 -= t0;
    printf("calc time = %d.%06d sec\n", t1 / MEGA, t1 % MEGA);
    printf("worker time = %d.%06d sec, operation count = %d\n", work_time / MEGA, work_time % MEGA, work_count);
    printf("wait time: %d.%06d sec\n", work_wait / MEGA, work_wait % MEGA);
    work_time = 0;
    work_count = 0;
    work_wait = 0;

    emp_deinit(&pi);
}

static void main_task(void *p)
{
    const struct FORMULA *fo = (struct FORMULA *)p;

    while (fo->name) {
        calc_pi(fo++);
    }
    printf("\ndone!\n");

    vTaskDelete(NULL);
}

void task_init(void)
{
    s_queue = xQueueCreate(SENDQ_LEN, sizeof(struct SEND_ITEM));
    r_queue = xQueueCreate(RECEIVE_LEN, sizeof(struct RECEIVE_ITEM));
    if (!s_queue || !r_queue) {
	ESP_LOGE(TAG, "xQueueCreate fail");
	exit(1);
    }

#ifdef BLINK_LED
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
#endif

    if ((xTaskCreatePinnedToCore(work_task, "work task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL, 1) != pdPASS)
    	|| (xTaskCreatePinnedToCore(main_task, "main task", 4096, (void *)&formula[0], tskIDLE_PRIORITY + 1, NULL, 0) != pdPASS)) {
	ESP_LOGE(TAG, "xTaskCreate fail");
	exit(1);
    }
}

void app_main(void)
{
    // create tasks
    task_init();
}
