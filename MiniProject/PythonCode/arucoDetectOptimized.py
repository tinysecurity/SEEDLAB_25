import time
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import numpy as np
import cv2
from cv2 import aruco
from smbus2 import SMBus

lcd_columns = 16
lcd_rows = 2

i2c = board.I2C()

lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns,lcd_rows)

