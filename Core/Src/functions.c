/*
 * Functions.c
 *
 *  Created on: Apr 8, 2020
 *      Author: Admin
 */
#include "functions.h"
#include "main.h"
#include <stdlib.h>
#include <stdio.h>
#define ClearBit(A,k)   (A &= ~(1UL << k))
#define SetBit(A,k)    (A |= 1UL << k)
#define TestBit(A,k) ((A >> k) & 1UL)
#define ToggleBit(A,k)  ( A ^= 1UL << k)
#define Flash_size 7



//extern int16_t num_diplay; // число для отображения
extern uint8_t Temperature_error; // счетчик ошибок измерения температуры
extern int16_t Set_add; // коррекция
extern int16_t Set_mul; // коррекция
extern uint8_t Set_hyst; // гистерезис
extern uint8_t Set_mode; // режим работы
extern uint16_t Set_temp; // задание температуры
extern uint8_t Set_delay; // минимальное время удержания контактов, сек
extern uint16_t Uvdda;
extern uint8_t Set_sens_type;
Flash_DataType Flash_memory;

void discr_reg (discr_reg_DataType *reg)
{
	if (reg->en!=0)
	{

		if ((reg->in > (reg->set + reg->hyst))&&(HAL_GetTick() > reg->time_privat + reg->min_time))
			{
			if (reg->mode) reg->out = 0;
			else reg->out = 1;
			}

		if (reg->in < (reg->set - reg->hyst))
			{
			if (reg->mode) reg->out = 1;
			else reg->out = 0;
			reg->time_privat = HAL_GetTick();
			}
	}
	else reg->out = 0;
}



void func_key (key_DataType* key)
{
//	key->key_enter = HAL_GPIO_ReadPin (KeyEnter_GPIO_Port, KeyEnter_Pin);
//	key->key_down = HAL_GPIO_ReadPin (KeyDown_GPIO_Port, KeyDown_Pin);
//	key->key_up = HAL_GPIO_ReadPin (KeyUp_GPIO_Port, KeyUp_Pin);

	//---------нажатие любой клавиши---------------------------
	key->any_key = key->key_enter | key->key_down | key->key_up;

	//-----------------------enter----------------------------
	if ((key->key_enter!=0) && !TestBit(key->R_trig_privat,0))//фронт нажатия, начало отсчета для подавления дребезга
	{
		SetBit(key->R_trig_privat, 0);
		key->time_privat[0] = HAL_GetTick();
	}
	if ((key->key_enter!=0) && (20 < HAL_GetTick() - key->time_privat[0]))
	{
		key->key_enter = HAL_GetTick() - key->time_privat[0];
	}
	if (key->key_enter==0) // кнопка не нажата
	{
		ClearBit(key->R_trig_privat, 0); //сбрасываем детектор фронта нажатия кнопки
		key->time_privat[0]=0;
	}
	if (key->time_privat[0] == 0 && key->time_privat[4] != 0) key->key_enter_fall = 1, key->key_enter_rise = 0;
	if (key->time_privat[0] == 0 && key->time_privat[4] == 0) key->key_enter_fall = 0, key->key_enter_rise = 0;
	if (key->time_privat[0] != 0 && key->time_privat[4] == 0) key->key_enter_rise = 1, key->key_enter_fall = 0;
	if (key->time_privat[0] != 0 && key->time_privat[4] != 0) key->key_enter_rise = 0, key->key_enter_fall = 0;
	key->time_privat[4] = key->time_privat[0]; //save last value

	//-----------------------down----------------------------
	if ((key->key_down!=0) && !TestBit(key->R_trig_privat,2))
	{
		SetBit(key->R_trig_privat, 2);
		key->time_privat[2] = HAL_GetTick();
	}
	if ((key->key_down!=0) && (20 < HAL_GetTick() - key->time_privat[2]))
	{
		key->key_down = HAL_GetTick() - key->time_privat[2];
	}
	if (key->key_down==0)
	{
		ClearBit(key->R_trig_privat, 2);
		key->time_privat[2]=0;
	}
	if (key->time_privat[2] == 0 && key->time_privat[6] != 0) key->key_down_fall = 1, key->key_down_rise = 0;
	if (key->time_privat[2] == 0 && key->time_privat[6] == 0) key->key_down_fall = 0, key->key_down_rise = 0;
	if (key->time_privat[2] != 0 && key->time_privat[6] == 0) key->key_down_rise = 1, key->key_down_fall = 0;
	if (key->time_privat[2] != 0 && key->time_privat[6] != 0) key->key_down_rise = 0, key->key_down_fall = 0;
	key->time_privat[6] = key->time_privat[2]; //save last value

	//-----------------------up----------------------------
	if ((key->key_up!=0) && !TestBit(key->R_trig_privat,3)) //нажата кнопка , определение фронта
	{
		SetBit(key->R_trig_privat, 3); // устанавливаем флаг
		key->time_privat[3] = HAL_GetTick(); // запись начального времени
	}
	if ((key->key_up!=0) && (20 < HAL_GetTick() - key->time_privat[3])) // если время нажатия превысило на 20мс
	{
		key->key_up = HAL_GetTick() - key->time_privat[3]; // сохраняем время нажатия кнопки
	}
	if (key->key_up==0) // если кнопка отпущена то все сбрасываем
	{
		ClearBit(key->R_trig_privat, 3);
		key->time_privat[3] = 0;
	}
	if (key->time_privat[3] == 0 && key->time_privat[7] != 0) key->key_up_fall = 1, key->key_up_rise = 0;
	if (key->time_privat[3] == 0 && key->time_privat[7] == 0) key->key_up_fall = 0, key->key_up_rise = 0;
	if (key->time_privat[3] != 0 && key->time_privat[7] == 0) key->key_up_rise = 1; key->key_up_fall = 0;
	if (key->time_privat[3] != 0 && key->time_privat[7] != 0) key->key_up_rise = 0, key->key_up_fall = 0;
	key->time_privat[7] = key->time_privat[3]; //save last value
}

void FLASH_Write(uint32_t* data, uint32_t adr, uint16_t length) {
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PAGEError = 0;

    HAL_FLASH_Unlock(); // Разблокируем флеш память

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.NbPages = 1;
    EraseInitStruct.Page = adr;

    HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError); // Очищаем страницу флеш памяти
//   uint32_t* ptr = data;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_FAST, adr, *data); // Записываем данные в флеш-память


    HAL_FLASH_Lock(); // Блокируем флеш память
}

int16_t limit (int16_t data, int16_t min, int16_t max)
{
	if (data < min) return min;
	if (data > max) return max;
	return data;
}

void Flash_memory_write (void)
{
	Flash_memory.Set_temp = Set_temp;
	Flash_memory.Set_hyst = Set_hyst;
	Flash_memory.Set_add = Set_add;
	Flash_memory.Set_mul = Set_mul;
	Flash_memory.Set_delay = Set_delay;
	Flash_memory.Set_sens_type = Set_sens_type;
	Flash_memory.Set_mode = Set_mode;
  FLASH_Write ((uint32_t *)&Flash_memory, (uint32_t) 0x08003C00, sizeof(Flash_memory)); // запись в последнюю страницу памяти
}

uint8_t timer(uint8_t copy, uint16_t time_off, uint16_t time_on ) // таймер циклический
{
	static uint32_t time_rec[10];// новая переменная
	if (time_rec[copy] == 0) //запись начального значения
			{
				time_rec[copy] = HAL_GetTick();
				return 1;// возврат 1
			}
	if (HAL_GetTick()>(time_rec[copy]+(uint32_t)time_on)) // глобальный счетчик больше чем время включения + последнее сохраненное время
		{
		if ((HAL_GetTick()>(time_rec[copy]+(uint32_t)time_on + (uint32_t)time_off))) time_rec[copy] = HAL_GetTick(); // выполнить условие если глобальный счетчик чем последнее сохраненное время +время включения+ время выключения
		return 0; // возврат 0
		}
	return 1; // возврат 1
}

// медиана на 3 значения со своим буфером
uint32_t median(uint32_t newVal) {
  static int buf[3];
  static uint8_t count = 0;
  buf[count] = newVal;
  if (++count >= 3) count = 0;
  return (max(buf[0], buf[1]) == max(buf[1], buf[2])) ? max(buf[0], buf[2]) : max(buf[1], min(buf[0], buf[2]));
}

int32_t filter(int32_t in, uint32_t coeff_A, uint32_t coeff_k ) {
	static uint32_t filt;

filt = (coeff_A * filt + in) >> coeff_k;
return filt;
}
