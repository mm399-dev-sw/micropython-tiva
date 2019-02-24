#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "py/runtime.h"
#include "py/stackctrl.h"
#include "py/gc.h"
#include "py/obj.h"
#include "py/mphal.h"
#include "lib/mp-readline/readline.h"
#include "lib/utils/pyexec.h"
//#include "lib/oofatfs/ff.h"
//#include "lwip/init.h"
//#include "extmod/vfs.h"
//#include "extmod/vfs_fat.h"





#include "py/compile.h"
#include "py/runtime.h"
#include "py/repl.h"
#include "py/mperrno.h"
#include "mods/modmachine.h"
#include "mods/pin.h"
#include "mods/uart.h"
#include "mods/systick.h"
//#include "mods/pybpin.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_hibernate.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

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

//extern uint32_t pui32Stack[128];
extern char * __StackTop;
extern char * _heap_start;
extern char *_heap_end;
//static char *stack_top;
#if MICROPY_ENABLE_GC
//static char heap[2048];
#endif

int main(int argc, char **argv) {
//    int stack_dummy;
    mp_stack_ctrl_init();

    mp_stack_set_top(&__StackTop);
    //mp_stack_set_limit((char*)&_estack - (char*)&_heap_end - 1024);

    #if MICROPY_ENABLE_GC
    gc_init(&_heap_start, &_heap_end);
    #endif

    // Python threading init
#if MICROPY_PY_THREAD
    mp_thread_init();
#endif

    machine_init();

    mp_init();

    readline_init0();
    pin_init0();
    uart_init0();

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

//void gc_collect(void) {
//    // WARNING: This gc_collect implementation doesn't try to get root
//    // pointers from CPU registers, and thus may function incorrectly.
//    void *dummy;
//    gc_collect_start();
//    gc_collect_root(&dummy, ((mp_uint_t)stack_top - (mp_uint_t)&dummy) / sizeof(mp_uint_t));
//    gc_collect_end();
//    gc_dump_info();
//}

//void gc_collect(void) {
//
//    gc_collect_start();
//    // get the registers and the sp
//    uintptr_t regs[10];
//    uintptr_t sp = gc_helper_get_regs_and_sp(regs);
//
//    // trace the stack, including the registers (since they live on the stack in this function)
//    #if MICROPY_PY_THREAD
//    gc_collect_root((void**)sp, ((uint32_t)MP_STATE_THREAD(stack_top) - sp) / sizeof(uint32_t));
//    #else
//    gc_collect_root((void**)sp, ((uint32_t)&_ram_end - sp) / sizeof(uint32_t));
//    #endif
//
//    // trace root pointers from any threads
//    #if MICROPY_PY_THREAD
//    mp_thread_gc_others();
//    #endif
//
//    // end the GC
//    gc_collect_end();
//    gc_dump_info();
//}

//mp_lexer_t *mp_lexer_new_from_file(const char *filename) {
//    mp_raise_OSError(MP_ENOENT);
//}

//mp_import_stat_t mp_import_stat(const char *path) { // @suppress("Type cannot be resolved")
//    return MP_IMPORT_STAT_NO_EXIST;
//}

//mp_obj_t mp_builtin_open(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
//    return mp_const_none;
//}
//MP_DEFINE_CONST_FUN_OBJ_KW(mp_builtin_open_obj, 1, mp_builtin_open);

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

#if MICROPY_MIN_USE_CORTEX_CPU
void _start(void) {
    // when we get here: stack is initialised, bss is clear, data is copied

    // SCB->CCR: enable 8-byte stack alignment for IRQ handlers, in accord with EABI
    *((volatile uint32_t*)0xe000ed14) |= 1 << 9;

    // initialise the cpu and peripherals
    #if MICROPY_MIN_USE_TM4C123_MCU
    void tm4c123_init(void);
    tm4c123_init();
    #endif

    // now that we have a basic system up and running we can call main
    main(0, NULL);

    // we must not return
    for (;;) {
    }
}

#endif

#if MICROPY_MIN_USE_TM4C123_MCU

//typedef struct {
//    volatile uint32_t DID0;
//    volatile uint32_t DID1;
//    uint32_t _1[10];
//    volatile uint32_t PBORCTL;
//    uint32_t _2[7];
//    volatile uint32_t RIS;
//    volatile uint32_t IMC;
//    volatile uint32_t MISC;
//    volatile uint32_t RESC;
//    volatile uint32_t RCC;
//    uint32_t _3[2];
//    volatile uint32_t GPIOHBCTL;
//    volatile uint32_t RCC2;
//    uint32_t _4[2];
//    volatile uint32_t MOSCCTL;
//    uint32_t _5[49];
//    volatile uint32_t DSLPCLKCFG;
//    uint32_t _6;
//    volatile uint32_t SYSPROP;
//    volatile uint32_t PIOSCCAL;
//    volatile uint32_t PIOSCSTAT;
//    uint32_t _7[2];
//    volatile uint32_t PLLFREQ0;
//    volatile uint32_t PLLFREQ1;
//    volatile uint32_t PLLSTAT;
//    uint32_t _8[7];                 // 0x16C - 0x187
//    uint32_t LPCR[18];              // LPWR Config Registers 0x188 - 0x1CF
//    uint32_t _9[76];                // 0x1D0 - 0x2FF
//    uint32_t PPR[24];               // Peripheral Present Registers 0x300 - 0x35F
//    uint32_t _10[104];              // 0x360 - 0x4FF
//    uint32_t PRR[24];               // Peripheral Reset Registers 0x500 - 0x55F
//    uint32_t _11[40];               // 0x560 - 0x5FF
//    volatile uint32_t RCGCWD;       // Peripheral Run Mode Clock Gating Control begin
//    volatile uint32_t RCGCTIMER;
//    volatile uint32_t RCGCGPIO;
//    volatile uint32_t RCGCDMA;
//    uint32_t _12;
//    volatile uint32_t RCGCHIB;
//    volatile uint32_t RCGCUART;
//    volatile uint32_t RCGCSSI;
//    volatile uint32_t RCGCI2C;
//    uint32_t _13;
//    volatile uint32_t RCGCUSB;
//    uint32_t _14[2];
//    volatile uint32_t RCGCCAN;
//    volatile uint32_t RCGCADC;
//    volatile uint32_t RCGCACMP;
//    volatile uint32_t RCGCPWM;
//    volatile uint32_t RCGCQEI;
//    uint32_t _15[4];
//    volatile uint32_t RCGCEEPROM;
//    volatile uint32_t RCGCWTIMER;
//    uint32_t _16[40];               // 0x660 - 0x6FF
//    uint32_t SCGC[24];              // Peripheral Sleep Mode Clock Gating Control
//    uint32_t _17[40];               // 0x760 - 0x7FF
//    uint32_t DCGC[24];              // Peripheral Deep-Sleep Mode Clock Gating Control
//    uint32_t _18[104];              // 0x860 - 0x9FF
//    volatile uint32_t PRWD;         // Peripheral Ready begin
//    volatile uint32_t PRTIMER;
//    volatile uint32_t PRGPIO;
//    volatile uint32_t PRDMA;
//    uint32_t _19;
//    volatile uint32_t PRHIB;
//    volatile uint32_t PRUART;
//    volatile uint32_t PRSSI;
//    volatile uint32_t PRI2C;
//    uint32_t _20;
//    volatile uint32_t PRUSB;
//    uint32_t _21[2];
//    volatile uint32_t PRCAN;
//    volatile uint32_t PRADC;
//    volatile uint32_t PRACMP;
//    volatile uint32_t PRPWM;
//    volatile uint32_t PRQEI;
//    uint32_t _22[4];
//    volatile uint32_t PREEPROM;
//    volatile uint32_t PRWTIMER;
//} periph_sysctl_t;





#define UART0  ((periph_uart_t*) 0x4000C000)

//#define SYSCTL ((periph_sysctl_t*)  0x400FE000)

void SysTick_Handler(void);

void tm4c123_init(void) {
    // basic MCU config

    // set system clock to 80MHz
    SYSCTL->RCC |= (uint32_t)0x00000800;   // set BYPASS bit
    SYSCTL->RCC2 |= (uint32_t)0xC0000800;  // set BYPASS2 bit, DIV400 and USERCC2
    SYSCTL->RCC &= (uint32_t)0xFFBFFFFF;   // clear USESYSDIV bit
    SYSCTL->RCC = (SYSCTL->RCC & (uint32_t)0xFFFFF83F) | (uint32_t)0x00000B70;    // XTAL to 16 MHz
    SYSCTL->MISC &= 0xFFFFFFBF;            // clear PLLLRIS bit
    SYSCTL->RCC2 &= (uint32_t)0xFFFFDFFF;  // clear PWRDN2 Bit to enable PLL
    SYSCTL->RCC2 = (SYSCTL->RCC2 & (uint32_t)0xE03FFFFF) | (uint32_t)0x01000000;  // Set SYSDIV2 to 0x2 for 80MHz
    SYSCTL->RCC |= (uint32_t)0x00400000;   // set USESYSDIV bit
    while(!(SYSCTL->RIS & 0x00000040)){};  // wait for Pll to lock, PLLLRIS bit
    SYSCTL->RCC2 &= 0xFFFFF7FF;            // clear BYPASS2 bit, clears BYPASS as well
    // write final configuration
    SYSCTL->RCC = (uint32_t)(0x07C00550);  // 0b0000 0 1111 1 0 0 000 000 0 0 0 10101 01 000 0
    SYSCTL->RCC2 = (uint32_t)(0xC1000000); // 0b 1100 0001 0000 0000 0000 0000 0000 0000

    // enable high performance GPIO BUS Ctl
    SYSCTL->GPIOHBCTL = 0x0000003F;

    // enable GPIO Port F
        SYSCTL->RCGCGPIO |= 0x00000020;
    //*((volatile uint32_t*)0x400FE608) = 0x00000021;
    while( !(SYSCTL->PRGPIO & 0x00000020)){};
    //while( *((volatile uint32_t*)0x400FEA08) != 0x00000021){};

//    // turn on an LED! (on pyboard it's the red one)
//    gpio_init(GPIOF, 1, GPIO_MODE_OUT, GPIO_PULL_NONE, 0);
//    gpio_init(GPIOF, 2, GPIO_MODE_OUT, GPIO_PULL_NONE, 0);
//    gpio_init(GPIOF, 3, GPIO_MODE_OUT, GPIO_PULL_NONE, 0);
//
//    gpio_high(GPIOF, 3);
//    //*((volatile uint32_t*)0x4005D3FC) = 0x00000007;

    // enable UART0 at 9600 baud (TX=A1, RX=A0)
    // enable UART0
    SYSCTL->RCGCUART = 0x00000001;
    //*((volatile uint32_t*)0x400FE618) = 0x00000001;
    while(SYSCTL->PRUART != 0x00000001){};
    //while( *((volatile uint32_t*)0x400FEA18) != 0x00000001){};

    // enable GPIO Port A
    SYSCTL->RCGCGPIO |= 0x00000001;
    while( !(SYSCTL->PRGPIO & 0x00000001)){};

//    // GPIOA already configured for UART0 after reset
//    gpio_init(GPIOA, 0, GPIO_MODE_ALT, GPIO_PULL_NONE, 1);
//    gpio_init(GPIOA, 1, GPIO_MODE_ALT, GPIO_PULL_NONE, 1);

    MAP_GPIOPinTypeUART(GPIO_PORTA_AHB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // BRD = IBRD + FRAC = UARTSysClk / (ClkDiv * BaudRate)
    // 520.8333 = 104  + 0.166 = 80MHz      / (16     * 9600    )
    // FBRD = (0.833333 * 64 + 0.5) ~ 53
    UART0->CTL &= ~0x00000001;
    UART0->IBRD = 0x00000208;
    UART0->FBRD = 0x00000035;
    UART0->LCRH = 0x00000060; // no stick parity, word length 8bit, FIFO enable, one STOP bit, odd parity, no parity check, no break
    UART0->CC = 0x00000000;   // use SysClock
    UART0->CTL = 0x00000300; // disable cts & rts, RXE, TXE, no loopback, 16x oversampling, TXRIS on IFLS match, no smart card, no low power, no SIR, UART enabled
    UART0->CTL |= 0x00000001;
    // to change settings in active mode: page 918 of reference


    //Setup of Systick to 1ms
    //Already registered in int vector
//    SysTickIntDisable();
//    SysTickIntRegister(SysTick_Handler);
    SysTickIntEnable();
    SysTickPeriodSet(SysCtlClockGet()/1000);
    SysTickEnable();
}

#endif
