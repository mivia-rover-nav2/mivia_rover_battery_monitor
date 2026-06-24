/**
 ******************************************************************************
 * @file    I2C_screen.h
 * @brief   Driver SSD1306 128x64 OLED via I2C pour STM32 HAL
 *          Implémentation autonome sans dépendance Arduino/Adafruit
 ******************************************************************************
 */

#ifndef I2C_SCREEN_H
#define I2C_SCREEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* =========================================================
 * Dimensions de l'écran
 * ========================================================= */
#define SSD1306_WIDTH   128
#define SSD1306_HEIGHT  64

/* =========================================================
 * Adresse I2C par défaut du SSD1306
 * ========================================================= */
#define SSD1306_I2C_ADDR  0x3C


/* =========================================================
 * Commandes SSD1306
 * ========================================================= */
#define SSD1306_MEMORYMODE          0x20
#define SSD1306_COLUMNADDR          0x21
#define SSD1306_PAGEADDR            0x22
#define SSD1306_SETCONTRAST         0x81
#define SSD1306_CHARGEPUMP          0x8D
#define SSD1306_SEGREMAP            0xA0
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_NORMALDISPLAY       0xA6
#define SSD1306_SETMULTIPLEX        0xA8
#define SSD1306_DISPLAYOFF          0xAE
#define SSD1306_DISPLAYON           0xAF
#define SSD1306_COMSCANDEC          0xC8
#define SSD1306_SETDISPLAYOFFSET    0xD3
#define SSD1306_SETDISPLAYCLOCKDIV  0xD5
#define SSD1306_SETPRECHARGE        0xD9
#define SSD1306_SETCOMPINS          0xDA
#define SSD1306_SETVCOMDETECT       0xDB
#define SSD1306_SETSTARTLINE        0x40
#define SSD1306_SWITCHCAPVCC        0x02

/* =========================================================
 * Couleurs
 * ========================================================= */
#define SSD1306_BLACK   0
#define SSD1306_WHITE   1

/* =========================================================
 * Taille de la police 5x7
 * ========================================================= */
#define FONT_WIDTH  6   /* 5 pixels + 1 espace */
#define FONT_HEIGHT 8   /* 1 page = 8 pixels */

/* =========================================================
 * Fonctions publiques
 * ========================================================= */

/**
 * @brief  Initialise le SSD1306
 * @param  hi2c  Pointeur vers le handle I2C HAL
 * @retval HAL_OK si succès, HAL_ERROR sinon
 */
HAL_StatusTypeDef SSD1306_Init(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Efface le buffer d'affichage (met tous les pixels à 0)
 */
void SSD1306_ClearBuffer(void);

/**
 * @brief  Envoie le buffer vers l'écran via I2C
 * @param  hi2c  Pointeur vers le handle I2C HAL
 */
void SSD1306_Display(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Allume un pixel dans le buffer
 * @param  x      Colonne (0..127)
 * @param  y      Ligne   (0..63)
 * @param  color  SSD1306_WHITE ou SSD1306_BLACK
 */
void SSD1306_DrawPixel(int16_t x, int16_t y, uint8_t color);

/**
 * @brief  Affiche un caractère ASCII à la position courante
 * @param  hi2c  Pointeur vers le handle I2C HAL (non utilisé ici, pour compatibilité)
 * @param  x     Colonne de départ en pixels
 * @param  y     Ligne de départ en pixels
 * @param  c     Caractère à afficher
 * @param  size  Facteur d'échelle (1 = normal, 2 = double, ...)
 * @param  color SSD1306_WHITE ou SSD1306_BLACK
 */
void SSD1306_DrawChar(int16_t x, int16_t y, char c, uint8_t size, uint8_t color);

/**
 * @brief  Affiche une chaîne de caractères
 * @param  x     Colonne de départ en pixels
 * @param  y     Ligne de départ en pixels
 * @param  str   Chaîne terminée par '\0'
 * @param  size  Facteur d'échelle (1 = normal, 2 = double, ...)
 * @param  color SSD1306_WHITE ou SSD1306_BLACK
 */
void SSD1306_DrawString(int16_t x, int16_t y, const char *str, uint8_t size, uint8_t color);

/**
 * @brief  Dessine un rectangle plein (barre de batterie)
 * @param  x, y      Coin supérieur gauche
 * @param  w, h      Largeur et hauteur
 * @param  color     SSD1306_WHITE ou SSD1306_BLACK
 */
void SSD1306_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color);

/**
 * @brief  Dessine un rectangle vide (contour)
 * @param  x, y      Coin supérieur gauche
 * @param  w, h      Largeur et hauteur
 * @param  color     SSD1306_WHITE ou SSD1306_BLACK
 */
void SSD1306_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color);

#ifdef __cplusplus
}
#endif

#endif /* I2C_SCREEN_H */
