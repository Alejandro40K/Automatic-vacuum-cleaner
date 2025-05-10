#ifndef LCD_H
#define LCD_H

// BIBLIOTECAS
#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

//DEFINIMOS VARIABLES EXTERNAS
extern LiquidCrystal_I2C lcd;

//DECLARAMOS FUNCIONES 
void iniciarLCD();

#endif