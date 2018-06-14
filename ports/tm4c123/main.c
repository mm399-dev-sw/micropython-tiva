#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "py/compile.h"
#include "py/runtime.h"
#include "py/repl.h"
#include "py/gc.h"
#include "py/mperrno.h"
#include "lib/utils/pyexec.h"


#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ssi.h"
#include "inc/hw_adc.h"
#include "driverlib/rom.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"

#if MICROPY_ENABLE_COMPILER
void do_str(const char *src, mp_parse_input_kind_t input_kind) {
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_lexer_t *lex = mp_lexer_new_from_str_len(MP_QSTR__lt_stdin_gt_, src, strlen(src), 0);
        qstr source_name = lex->source_name;
        mp_parse_tree_t parse_tree = mp_parse(lex, input_kind);
        mp_obj_t module_fun = mp_compile(&parse_tree, source_name, MP_EMIT_OPT_NONE, true);
        mp_call_function_0(module_fun);
        nlr_pop();
    } else {
        // uncaught exception
        mp_obj_print_exception(&mp_plat_print, (mp_obj_t)nlr.ret_val);
    }
}
#endif

static char *stack_top;
#if MICROPY_ENABLE_GC
static char heap[2048];
#endif

int main(int argc, char **argv) {
    int stack_dummy;
    stack_top = (char*)&stack_dummy;

    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |SYSCTL_XTAL_16MHZ);

//Pin1: red, Pin2: Blue, Pin3: Green
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 1);

    #if MICROPY_ENABLE_GC
    gc_init(heap, heap + sizeof(heap));
    #endif
    mp_init();
    #if MICROPY_ENABLE_COMPILER
    #if MICROPY_REPL_EVENT_DRIVEN
    pyexec_event_repl_init();
    for (;;) {
        int c = mp_hal_stdin_rx_chr();
        if (pyexec_event_repl_process_char(c)) {
            break;
        }
    }
    #else
    pyexec_friendly_repl();
    #endif
    //do_str("print('hello world!', list(x+1 for x in range(10)), end='eol\\n')", MP_PARSE_SINGLE_INPUT);
    //do_str("for i in range(10):\r\n  print(i)", MP_PARSE_FILE_INPUT);
    #else
    pyexec_frozen_module("frozentest.py");
    #endif
    mp_deinit();
    return 0;
}

void gc_collect(void) {
    // WARNING: This gc_collect implementation doesn't try to get root
    // pointers from CPU registers, and thus may function incorrectly.
    void *dummy;
    gc_collect_start();
    gc_collect_root(&dummy, ((mp_uint_t)stack_top - (mp_uint_t)&dummy) / sizeof(mp_uint_t));
    gc_collect_end();
    gc_dump_info();
}

mp_lexer_t *mp_lexer_new_from_file(const char *filename) {
    mp_raise_OSError(MP_ENOENT);
}

mp_import_stat_t mp_import_stat(const char *path) {
    return MP_IMPORT_STAT_NO_EXIST;
}

mp_obj_t mp_builtin_open(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(mp_builtin_open_obj, 1, mp_builtin_open);

void nlr_jump_fail(void *val) {
    while (1);
}

void NORETURN __fatal_error(const char *msg) {
    while (1);
}

#ifndef NDEBUG
void MP_WEAK __assert_func(const char *file, int line, const char *func, const char *expr) {
    printf("Assertion '%s' failed, at file %s:%d\n", expr, file, line);
    __fatal_error("Assertion failed");
}
#endif


/* void stm32_init(void) { */
/*     // basic MCU config */
/*     RCC->CR |= (uint32_t)0x00000001; // set HSION */
/*     RCC->CFGR = 0x00000000; // reset all */
/*     RCC->CR &= (uint32_t)0xfef6ffff; // reset HSEON, CSSON, PLLON */
/*     RCC->PLLCFGR = 0x24003010; // reset PLLCFGR */
/*     RCC->CR &= (uint32_t)0xfffbffff; // reset HSEBYP */
/*     RCC->CIR = 0x00000000; // disable IRQs */

/*     // leave the clock as-is (internal 16MHz) */

/*     // enable GPIO clocks */
/*     RCC->AHB1ENR |= 0x00000003; // GPIOAEN, GPIOBEN */

/*     // turn on an LED! (on pyboard it's the red one) */
/*     gpio_init(GPIOA, 13, GPIO_MODE_OUT, GPIO_PULL_NONE, 0); */
/*     gpio_high(GPIOA, 13); */

/*     // enable UART1 at 9600 baud (TX=B6, RX=B7) */
/*     gpio_init(GPIOB, 6, GPIO_MODE_ALT, GPIO_PULL_NONE, 7); */
/*     gpio_init(GPIOB, 7, GPIO_MODE_ALT, GPIO_PULL_NONE, 7); */
/*     RCC->APB2ENR |= 0x00000010; // USART1EN */
/*     USART1->BRR = (104 << 4) | 3; // 16MHz/(16*104.1875) = 9598 baud */
/*     USART1->CR1 = 0x0000200c; // USART enable, tx enable, rx enable */
/* } */
