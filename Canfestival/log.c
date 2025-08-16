/*
 *  log.c
 *
 *  Created on: 2019年10月6日
 *      Author: armwind.chen@foxmail.com
 *      QQ group: 85486140
 */
#include "log.h"
#include "canfestival_usart.h"
#include <stdarg.h>

static const char s_log_prefix[LOG_MAX] = {
    'N', //LOG_NONE
    'E', //LOG_ERROR
    'W', //LOG_WARN
    'I', //LOG_INFO
    'D', //LOG_DEBUG
    'V', //LOG_VERBOSE
};


typedef void (*F_putc)(char c);
F_putc xputc = (F_putc)uart2SendChar;

/*----------------------------------------------*/
/* Put a null-terminated string                 */
/*----------------------------------------------*/
void log_puts (                    /* Put a string to the default device */
    const char* str             /* Pointer to the string */
)
{
    while (*str)
        xputc(*str++);
}

/*----------------------------------------------*/
/* Formatted string output                      */
/*----------------------------------------------*/
/*  log_xprintf("%d", 1234);            "1234"
    xprintf("%6d,%3d%%", -200, 5);  "  -200,  5%"
    xprintf("%-6u", 100);           "100   "
    xprintf("%ld", 12345678L);      "12345678"
    xprintf("%04x", 0xA3);          "00a3"
    xprintf("%08LX", 0x123ABC);     "00123ABC"
    xprintf("%016b", 0x550F);       "0101010100001111"
    xprintf("%s", "String");        "String"
    xprintf("%-4s", "abc");         "abc "
    xprintf("%4s", "abc");          " abc"
    xprintf("%c", 'a');             "a"
    xprintf("%f", 10.0);            <xprintf lacks floating point support>
*/
static
void log_vprintf (
    const char* fmt,    /* Pointer to the format string */
    va_list arp         /* Pointer to arguments */
)
{
    unsigned int r, i, j, w, f;
    unsigned long v;
    char s[16], c, d, *p;
    for (;;) {
        c = *fmt++;                 /* Get a char */
        if (!c) break;              /* End of format? */
        if (c != '%') {             /* Pass through it if not a % sequense */
            xputc(c); continue;
        }
        f = 0;
        c = *fmt++;                 /* Get first char of the sequense */
        if (c == '0') {             /* Flag: '0' padded */
            f = 1; c = *fmt++;
        } else {
            if (c == '-') {         /* Flag: left justified */
                f = 2; c = *fmt++;
            }
        }
        for (w = 0; c >= '0' && c <= '9'; c = *fmt++)   /* Minimum width */
            w = w * 10 + c - '0';
        if (c == 'l' || c == 'L') { /* Prefix: Size is long int */
            f |= 4; c = *fmt++;
        }
        if (!c) break;              /* End of format? */
        d = c;
        if (d >= 'a') d -= 0x20;
        switch (d) {                /* Type is... */
        case 'S' :                  /* String */
            p = va_arg(arp, char*);
            for (j = 0; p[j]; j++) ;
            while (!(f & 2) && j++ < w) xputc(' ');
            log_puts(p);
            while (j++ < w) xputc(' ');
            continue;
        case 'C' :                  /* Character */
            xputc((char)va_arg(arp, int)); continue;
        case 'B' :                  /* Binary */
            r = 2; break;
        case 'O' :                  /* Octal */
            r = 8; break;
        case 'D' :                  /* Signed decimal */
        case 'U' :                  /* Unsigned decimal */
            r = 10; break;
        case 'X' :                  /* Hexdecimal */
            r = 16; break;
        default:                    /* Unknown type (passthrough) */
            xputc(c); continue;
        }
        /* Get an argument and put it in numeral */
        v = (f & 4) ? va_arg(arp, long) : ((d == 'D') ? (long)va_arg(arp, int) : (long)va_arg(arp, unsigned int));
        if (d == 'D' && (v & 0x80000000)) {
            v = 0 - v;
            f |= 8;
        }
        i = 0;
        do {
            d = (char)(v % r); v /= r;
            if (d > 9) d += (c == 'x') ? 0x27 : 0x07;
            s[i++] = d + '0';
        } while (v && i < sizeof(s));
        if (f & 8) s[i++] = '-';
        j = i; d = (f & 1) ? '0' : ' ';
        while (!(f & 2) && j++ < w) xputc(d);
        do xputc(s[--i]); while(i);
        while (j++ < w) xputc(' ');
    }
}

static void log_xprintf (          /* Put a formatted string to the default device */
    const char* fmt,    /* Pointer to the format string */
    ...                 /* Optional arguments */
)
{
    va_list arp;
    va_start(arp, fmt);
    log_vprintf(fmt, arp);
    va_end(arp);
}

void log_printf(const int log_level,const char * tag, const char *func, const int line, const char *fmt, ...)
{
    va_list args;
    log_xprintf("%-10s [%c] %s:%d ", tag, s_log_prefix[log_level], func, line);
    va_start(args, fmt);
    log_vprintf(fmt, args);
    va_end(args);
    xputc('\n');
}

//#endif /* _USE_XFUNC_IN */
