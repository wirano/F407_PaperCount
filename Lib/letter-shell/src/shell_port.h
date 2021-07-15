//
// Created by Wirano on 2020/8/26.
//

#ifndef _SHELL_PORT_H
#define _SHELL_PORT_H


#include "shell.h"
#include "log.h"
#include "main.h"
#include <stdbool.h>


extern Shell shell;


/**
 * @brief shell读取数据函数原型
 *
 * @param data shell读取的字符
 * @param len 请求读取的字符数量
 *
 * @return unsigned short 实际读取到的字符数量
 */
typedef unsigned short (*shellRead)(char *data, unsigned short len);

/**
 * @brief shell写数据函数原型
 *
 * @param data 需写的字符数据
 * @param len 需要写入的字符数
 *
 * @return unsigned short 实际写入的字符数量
 */
typedef unsigned short (*shellWrite)(char *data, unsigned short len);

void uartLogWrite(char *buffer, short len);

void shell_ported_init(Shell *shell);

#endif //_SHELL_PORT_H
