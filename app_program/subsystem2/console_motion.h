#ifndef CONSOLE_MOTION_H
#define CONSOLE_MOTION_H

#include <stdbool.h>
#include <stdint.h>

/* Generic weak USB port hook: fixed integration name. */
void tm_console_receive_byte(uint8_t byte);
void console_motion_report(void);
int32_t console_motion_getchar(void);
/* A complete NUL-terminated line, at most 63 bytes. */
bool console_motion_execute(const char * p_line);

#endif /* CONSOLE_MOTION_H */
/* End of file. */
