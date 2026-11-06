#ifndef BASE_APP_H
#define BASE_APP_H

/* 应用层入口：由 main.c 在初始化后调用 base_app_init() 一次，
 * 再在 while(1) 中高频调用 base_app_poll()。
 * UART 和 CAN 的 HAL 中断回调也在 base_app.c 中实现。 */
void base_app_init(void);
void base_app_poll(void);
#endif
