/**********************************************************
 * Header file for LCD.c file 			Date: 26-04-2014
 * Created by 
 * Advanced Electronics Systems, Bengaluru.
 * Declared the functions and variables related to .c file
 **********************************************************/
#ifndef __LCD_H
#define __LCD_H

#define RS_CTRL  0x02000000  //P3.25
#define RW_CTRL  0x04000000  //P3.26
#define EN_CTRL  0x10000000  //P4.28
#define DT_CTRL  0x00F00000  //P1.20 to P1.23

void lcd_init(void);
void wr_cn(void);
void clr_disp(void);
void delay_lcd(unsigned int);
void lcd_com(void);						   
void wr_dn(void);
void lcd_data(void);
void clear_ports(void);
void lcd_puts(unsigned char *);

extern unsigned long int temp1 , temp2;

#endif

