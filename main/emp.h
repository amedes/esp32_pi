/* emp - multiple precision library for ESP32

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
typedef struct EMP {
    uint32_t *word;
    uint16_t size;
    uint16_t top;
    uint16_t bottom;
} emp_t;

void emp_init(emp_t *p, uint16_t size);
void emp_deinit(emp_t *dst);
void emp_add(emp_t *dst, emp_t *src);
void emp_sub(emp_t *dst, emp_t *src);
void emp_mul(emp_t *dst, emp_t *src, uint32_t m);
void emp_div(emp_t *dst, emp_t *src, uint32_t d);
void emp_srl(emp_t *dst, emp_t *src, int shift);
void emp_setint(emp_t *dst, uint32_t num);
void emp_printhex(emp_t *src);
void emp_printdec(emp_t *src);
void emp_copy(emp_t *dst, emp_t *src);
void emp_clear(emp_t *dst);
