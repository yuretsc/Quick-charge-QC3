/*
 * Functions.h
 *
 *  Created on: Apr 8, 2020
 *      Author: Admin
 */

#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_

#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))



#define ClearBit(A,k)   (A &= ~(1UL << k))
#define SetBit(A,k)    (A |= 1UL << k)
#define TestBit(A,k) ((A >> k) & 1UL)
#define ToggleBit(A,k)  ( A ^= 1UL << k)

#include "stm32g0xx_hal.h"
#include "main.h"
// обработка кнопок
typedef struct
{
	uint32_t key_enter; /* разрешение */
	uint32_t key_up; /*входная величина*/
	uint32_t key_down; /*задание*/
	uint32_t time_privat[8]; /*время нажатия мс*/
	uint32_t R_trig_privat;
	uint8_t key_enter_fall; /* отпускание кнопки */
	uint8_t key_up_fall; /* отпускание кнопки */
	uint8_t key_down_fall; /*отпускание кнопки*/
	uint8_t key_enter_rise; /* отпускание кнопки */
	uint8_t key_up_rise; /* отпускание кнопки */
	uint8_t key_down_rise; /*отпускание кнопки*/
	uint8_t any_key; /*нажата любая кнопка*/
} key_DataType;

// регулятор дискретный
typedef struct
{
	uint8_t out; /* разрешение */
	uint8_t en; /* разрешение */
	uint8_t mode; /*режимо работы 1- нагрев, 0 - охлаждение */
	int16_t in; /*входная величина*/
	int16_t set; /*задание*/
	uint16_t hyst; /*гистерезис*/
	uint32_t min_time; /*минимальный сигнал в mс*/
	uint32_t time_privat; /*время вызова мс*/
} discr_reg_DataType;

// ошибки
typedef struct
{
	uint8_t code; // код ошибки
	uint8_t mode; // режим работы регулятора 1 - нагрев, 0 - охлаждение
	int16_t T1_H; /* перегрев Т1 */
	int16_t T2_H; /* перегрев Т2 */
	int16_t T3_H; /* перегрев Т3 */
	int16_t T4_H; /* перегрев Т4 */
	int16_t T1_memory_privat; /*сохраненное значение Т1*/
	int16_t T2_memory_privat; /*сохраненное значение Т2*/
	int16_t T3_memory_privat; /*сохраненное значение Т3*/
	int16_t T4_memory_privat; /*сохраненное значение Т4*/
	int16_t R_trig_privat; /*триггер*/
	uint32_t dtime_privat; /*время сравнение прироста температуры мс*/
	uint32_t etime_privat; /*время длительности ошибки мс*/
	uint32_t time_privat[16];//*время для сохранения ошибки мс*/
} error_DataType;

typedef struct
{
	int16_t Set_add;
	int16_t Set_mul;
	int16_t Set_temp;
	uint8_t Set_hyst;
	uint8_t Set_delay;
	uint8_t Set_sens_type;
	uint8_t Set_mode;

} Flash_DataType;

void discr_reg (discr_reg_DataType*);
void func_key (key_DataType*);
void FLASH_Write (uint32_t *data, uint32_t adr, uint16_t length);
int16_t limit (int16_t data, int16_t min, int16_t max);
void Flash_memory_write (void);
void Display_num_3(int32_t in, uint8_t SegDP_1, uint8_t SegDP_2, uint8_t SegDP_3);
uint8_t timer(uint8_t copy, uint16_t time_off, uint16_t time_on ); // таймер циклический
uint32_t median(uint32_t newVal);
int32_t filter(int32_t in, uint32_t coeff_A, uint32_t coeff_k );
/*коэффициенты выбираются так:
k = 1, 2, 3…
A = (2^k) – 1
[k=2 A=3], [k=3 A=7], [k=4 A=15], [k=5 A=31]…
*/
#endif /* INC_FUNCTIONS_H_ */
