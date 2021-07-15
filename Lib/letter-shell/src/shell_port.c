//
// Created by Wirano on 2020/8/26.
//

#include "shell_port.h"


char shellBuffer[512];


shellWrite ShellWrite(char *data, unsigned short len)
{
    while (len--) {
        while ((USART1->SR & 0X40U) == 0);
        USART1->DR = (uint8_t) (*data++);
    }

    return len;
}

void uartLogWrite(char *buffer, short len)
{
    extern Log slog;

    if (slog.shell) {
        shellWriteEndLine(slog.shell, buffer, len);
    }
}

void shell_ported_init(Shell *shell)
{
    shell->write = ShellWrite;
    shellInit(shell, shellBuffer, sizeof(shellBuffer));
}
