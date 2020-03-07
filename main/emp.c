/* emp - multiple precision library for ESP32

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <malloc.h>
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"

#include "emp.h"

#define TAG "emp"

#ifdef CONFIG_SUPPRESS_PI_DIGITS
#define SUPPRESS_MIDDLE_DIGITS 1
#endif

void emp_init(emp_t *p, uint16_t size)
{
    uint32_t *mem;

    mem = calloc(size, sizeof(uint32_t));
    if (mem == NULL) {
	ESP_LOGE(TAG, "malloc fail");
	exit(1);
    }

    p->word = mem;
    p->size = size;
    p->top = size - 1;
    p->bottom = 0;
}

void emp_deinit(emp_t *dst)
{
    if (dst->word) free(dst->word);
    dst->word = NULL;
    dst->size = 0;
}

void emp_add(emp_t *dst, emp_t *src)
{
    uint32_t *p = &dst->word[src->bottom];
    uint32_t *q = &src->word[src->bottom];
    uint32_t loop = src->top - src->bottom + 1;
    uint32_t carry = 0, t0, t1, t2;

    asm volatile(
		    "loopgtz	%6, %=1f\n\t"
		    "l32i	%2, %0, 0\n\t"	// *p
		    "l32i	%3, %1, 0\n\t"	// *q
		    "addi	%1, %1, 4\n\t"	// q++
		    "add	%5, %4, %2\n\t"	// *p + carry
		    "add	%5, %5, %3\n\t"	// *p + *q + carry
		    "s32i	%5, %0, 0\n\t"	// *p = sum
		    "addi	%0, %0, 4\n\t"	// p++
		    "bnez	%4, %=3f\n\t"	// if carry == 1
		    "movi	%4, 0\n\t"
		    "bgeu	%5, %2, %=2f\n\t" // if sum >= *p + carry
		    "movi	%4, 1\n\t"	// carry
		    "j		%=2f\n\t"
		    "%=3:\n\t"
		    "movi	%4, 0\n\t"
		    "bltu	%2, %5, %=2f\n\t" // if *p + carry < sum
		    "movi	%4, 1\n\t"	// carry
		    "%=2:\n\t"
		    "nop.n\n\t"
		    "%=1:\n\t"
		    : "+r"(p), "+r"(q), "=r"(t0), "=r"(t1), "+r"(carry), "=r"(t2)
		    : "r"(loop)
		);

    	if (carry && (src->top < dst->size - 1)) {
	    dst->word[src->top + 1] += 1;
	}
}

void emp_sub(emp_t *dst, emp_t *src)
{
    uint32_t *p = &dst->word[src->bottom];
    uint32_t *q = &src->word[src->bottom];
    uint32_t loop = src->top - src->bottom + 1;
    uint32_t borrow = 0, t0, t1, t2;

    asm volatile(
		    "loopgtz	%6, %=1f\n\t"
		    "l32i	%2, %0, 0\n\t"	// t0 = *p
		    "l32i	%3, %1, 0\n\t"	// t1 = *q
		    "addi	%1, %1, 4\n\t"	// q++
		    "sub	%5, %2, %4\n\t"	// t2 = t0 - borrow
		    "sub	%5, %5, %3\n\t"	// t2 -= t1
		    "s32i	%5, %0, 0\n\t"	// *p = t2
		    "addi	%0, %0, 4\n\t"	// p++
		    "bnez	%4, %=3f\n\t"	// branch if borrow == 1
		    "movi	%4, 0\n\t"	// borrow = 0
		    "bgeu	%2, %3, %=2f\n\t" // branch if t0 >= t1
		    "movi	%4, 1\n\t"	// borrow = 1
		    "j		%=2f\n\t"
		    "%=3:\n\t"
		    "movi	%4, 0\n\t"	// borrow = 0
		    "bltu	%3, %2, %=2f\n\t" // branch if t1 < t0
		    "movi	%4, 1\n\t"	// borrow = 1
		    "%=2:\n\t"
		    "nop.n\n\t"			// for loopback
		    "%=1:\n\t"
		    : "+r"(p), "+r"(q), "=r"(t0), "=r"(t1), "+r"(borrow), "=r"(t2)
		    : "r"(loop)
		);

    	if (borrow && (src->top < dst->size - 1)) {
	    dst->word[src->top + 1] -= 1;
	}
}

void emp_mul(emp_t *dst, emp_t *src, uint32_t m)
{
    uint32_t *p = &dst->word[dst->bottom] - 1;
    uint32_t *q = &src->word[src->bottom];
    uint32_t loop = src->top - src->bottom + 1;
    uint32_t high = 0, t0, t1;

    asm volatile(
		    "loopgtz	%5, %=1f\n\t"
		    "l32i	%3, %1, 0\n\t"	// *q
		    "addi	%1, %1, 4\n\t"  // q++
		    "addi	%0, %0, 4\n\t"	// p++
		    "mull	%4, %3, %6\n\t"	// * m
		    "muluh	%3, %3, %6\n\t"	// high = high(*q * m)
		    "add	%2, %4, %2\n\t"	// + high
		    "s32i	%2, %0, 0\n\t"	// *p = mult
		    "bgeu	%2, %4, %=2f\n\t" // if add >= mult
		    "addi	%3, %3, 1\n\t"	// high += carry, no overflow
		    "%=2:\n\t"
		    "mov	%2, %3\n\t"	// high = high + carry
		    "%=1:\n\t"
		    : "+r"(p), "+r"(q), "+r"(high), "=&r"(t0), "=&r"(t1)
		    : "r"(loop), "r"(m)

		);

    if ((high > 0) && (dst->top < dst->size - 1)) {
	dst->word[++dst->top] = high;
    }
}

/*
 * p[n] = q[n] / d
 */
void emp_div(emp_t *dst, emp_t *src, uint32_t d)
{
    if ((src->word[src->top] == 0) && (src->top > src->bottom)) src->top--;
    dst->top = src->top;
    dst->bottom = src->bottom;

    uint32_t *p = &dst->word[dst->top];
    uint32_t *q = &src->word[src->top];
    uint32_t r = 0, t0, t1;
    uint32_t n = src->top - src->bottom + 1;
    // reverse order, 32bit access
    asm volatile(
		    "ssai	16\n\t"			// SAR = 16
		    "loop	%6, %=f\n\t"
		    "l32i	%2, %1, 0\n\t"		// *q
		    "addi	%1, %1, -4\n\t"		// q--
		    "addi	%0, %0, -4\n\t"		// p--
		    "src	%3, %3, %2\n\t"		// t0 = remainder | high16(*q)
		    "quou	%4, %3, %5\n\t"		// quotient = t0 / d
		    "remu	%3, %3, %5\n\t"		// remainder = t0 % d
		    "slli	%2, %2, 16\n\t"		// low16(*q) << 16
		    "src	%3, %3, %2\n\t"		// temp = remainder | low16(*q)
		    "quou	%2, %3, %5\n\t"		// quotient = temp / d
		    "remu	%3, %3, %5\n\t"		// remainder = t1 % d
		    "slli	%2, %2, 16\n\t"		// quotient << 16
		    "src	%2, %4, %2\n\t"		// quotient_high | quotient_low
		    "s32i	%2, %0, 4\n\t"		// *p = quotient_high | quotient_low
		    "%=:\n\t"
		    : "+r"(p), "+r"(q), "=&r"(t1), "+r"(r), "=&r"(t0)
		    : "r"(d), "r"(n)
		);
}

void emp_srl(emp_t *dst, emp_t *src, int shift)
{
    uint32_t *p;
    uint32_t *q;
    uint32_t n;
    uint32_t t0, t1 = 0;
    
    if (shift == 0) return;
    if (shift < 0 || shift > 31) {
	ESP_LOGE(TAG, "shift out of range: %d", shift);
	exit(1);
    }

    //if ((src->word[src->top] == 0) && (src->bottom > 0)) src->top--;
    p = &dst->word[dst->top];
    q = &src->word[src->top];
    n = src->top - src->bottom + 1;

    n = (src->top / 2) - (src->bottom / 2) + 1;
    p = &dst->word[src->top & ~1];
    q = &src->word[src->top & ~1];
    dst->top = src->top;

    //printf("emp_srl: n = %d\n", n);
    //printf("emp_srl: shift = %d\n", shift);

    // 32bit access
    asm volatile(
		    "ssr	%5\n\t"
		    "loopgtz	%4, %=f\n\t"
		    "l32i	%2, %1, 4\n\t"		// t0 = *(q + 1)
		    "src	%3, %3, %2\n\t"		// t1 = (t1 | t0) >> shift
		    "s32i	%3, %0, 4\n\t"		// *(p + 1) = t1
		    "l32i	%3, %1, 0\n\t"		// t1 = *q 
		    "addi	%1, %1, -8\n\t"		// q -= 2
		    "addi	%0, %0, -8\n\t"		// p -= 2
		    "src	%2, %2, %3\n\t"		// t0 = (t0 | t1) >> shift
		    "s32i	%2, %0, 8\n\t"		// *p = t0
		    "%=:\n\t"
		    : "+r"(p), "+r"(q), "=r"(t0), "+r"(t1)
		    : "r"(n), "r"(shift)
		);
}

void emp_setint(emp_t *dst, uint32_t num)
{
    memset(dst->word, 0, dst->size * sizeof(uint32_t));
    dst->top = dst->size - 1;
    dst->bottom = 0;
    dst->word[dst->top] = num;
}

void emp_printhex(emp_t *src)
{
    for (int i = src->size - 1; i >= 0; --i) {
	printf("%08x ", src->word[i]);
    }
    printf("\n");
}

void emp_printdec(emp_t *src)
{
    uint32_t *p = &src->word[src->size - 1];
    int n = src->size;
    uint32_t d0, d1;
    int digits = 0;

    printf("%u.\n", *p);
    *p = 0;

    int m = (n - 1) * 32 * log(2) / (10 * log(10)) + 1; // number of digits under decimal point
    for  (int j = 0; j < m; j++) {

	emp_mul(src, src, 100000);
	d0 = *p;
	*p = 0;
	emp_mul(src, src, 100000);
	d1 = *p;
	*p = 0;
	digits += 10;

#ifdef SUPPRESS_MIDDLE_DIGITS
	if (j == 10) printf("\tpi digits suppressed... can change using menuconfig\n");
	if ((j < 10) || (j >= m - (10 + m % 10)))
#endif
	   printf("%05u%05u%c", d0, d1, (j % 10 == 9) ? '\n' : ' ');
    }
    printf("\n%d digits\n", digits);
}

void emp_copy(emp_t *dst, emp_t *src)
{
    for (int i = src->bottom; i <= src->top; i++) {
	dst->word[i] = src->word[i];
    }
    dst->top = src->top;
    dst->bottom = src->bottom;
}

void emp_clear(emp_t *dst)
{
    memset(dst->word, 0, dst->size * sizeof(uint32_t));
    dst->top = dst->size - 1;
    dst->bottom = 0;
}
