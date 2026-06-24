/**
 ******************************************************************************
 * @file    I2C_screen.c
 * @brief   Driver SSD1306 128x64 OLED via I2C pour STM32 HAL
 * Implémentation autonome sans dépendance Arduino/Adafruit
 ******************************************************************************
 *
 * Police bitmap 5x7 intégrée (ASCII 0x20 à 0x7E).
 * Le buffer est de 128*64/8 = 1024 octets.
 * Envoi en mode page-adressage horizontal (commande 0x00).
 *
 ******************************************************************************
 */

#include "I2C_screen.h"
#include <string.h>

/* =========================================================
 * Buffer interne (1 bit par pixel)
 * Organisation : 8 pages de 128 colonnes
 * buffer[page*128 + col] contient les 8 pixels verticaux
 * ========================================================= */
static uint8_t ssd1306_buffer[SSD1306_WIDTH * (SSD1306_HEIGHT / 8)];

/* =========================================================
 * Police 5x7 (ASCII 0x20 ' ' à 0x7E '~')
 * Chaque caractère est décrit par 5 octets (colonnes)
 * bit0 = ligne du haut, bit6 = ligne du bas
 * ========================================================= */
static const uint8_t font5x7[][5] = {
    { 0x00, 0x00, 0x00, 0x00, 0x00 }, /* 0x20  ' ' */
    { 0x00, 0x00, 0x5F, 0x00, 0x00 }, /* 0x21  '!' */
    { 0x00, 0x07, 0x00, 0x07, 0x00 }, /* 0x22  '"' */
    { 0x14, 0x7F, 0x14, 0x7F, 0x14 }, /* 0x23  '#' */
    { 0x24, 0x2A, 0x7F, 0x2A, 0x12 }, /* 0x24  '$' */
    { 0x23, 0x13, 0x08, 0x64, 0x62 }, /* 0x25  '%' */
    { 0x36, 0x49, 0x55, 0x22, 0x50 }, /* 0x26  '&' */
    { 0x00, 0x05, 0x03, 0x00, 0x00 }, /* 0x27  ''' */
    { 0x00, 0x1C, 0x22, 0x41, 0x00 }, /* 0x28  '(' */
    { 0x00, 0x41, 0x22, 0x1C, 0x00 }, /* 0x29  ')' */
    { 0x14, 0x08, 0x3E, 0x08, 0x14 }, /* 0x2A  '*' */
    { 0x08, 0x08, 0x3E, 0x08, 0x08 }, /* 0x2B  '+' */
    { 0x00, 0x50, 0x30, 0x00, 0x00 }, /* 0x2C  ',' */
    { 0x08, 0x08, 0x08, 0x08, 0x08 }, /* 0x2D  '-' */
    { 0x00, 0x60, 0x60, 0x00, 0x00 }, /* 0x2E  '.' */
    { 0x20, 0x10, 0x08, 0x04, 0x02 }, /* 0x2F  '/' */
    { 0x3E, 0x51, 0x49, 0x45, 0x3E }, /* 0x30  '0' */
    { 0x00, 0x42, 0x7F, 0x40, 0x00 }, /* 0x31  '1' */
    { 0x42, 0x61, 0x51, 0x49, 0x46 }, /* 0x32  '2' */
    { 0x21, 0x41, 0x45, 0x4B, 0x31 }, /* 0x33  '3' */
    { 0x18, 0x14, 0x12, 0x7F, 0x10 }, /* 0x34  '4' */
    { 0x27, 0x45, 0x45, 0x45, 0x39 }, /* 0x35  '5' */
    { 0x3C, 0x4A, 0x49, 0x49, 0x30 }, /* 0x36  '6' */
    { 0x01, 0x71, 0x09, 0x05, 0x03 }, /* 0x37  '7' */
    { 0x36, 0x49, 0x49, 0x49, 0x36 }, /* 0x38  '8' */
    { 0x06, 0x49, 0x49, 0x29, 0x1E }, /* 0x39  '9' */
    { 0x00, 0x36, 0x36, 0x00, 0x00 }, /* 0x3A  ':' */
    { 0x00, 0x56, 0x36, 0x00, 0x00 }, /* 0x3B  ';' */
    { 0x08, 0x14, 0x22, 0x41, 0x00 }, /* 0x3C  '<' */
    { 0x14, 0x14, 0x14, 0x14, 0x14 }, /* 0x3D  '=' */
    { 0x00, 0x41, 0x22, 0x14, 0x08 }, /* 0x3E  '>' */
    { 0x02, 0x01, 0x51, 0x09, 0x06 }, /* 0x3F  '?' */
    { 0x32, 0x49, 0x79, 0x41, 0x3E }, /* 0x40  '@' */
    { 0x7E, 0x11, 0x11, 0x11, 0x7E }, /* 0x41  'A' */
    { 0x7F, 0x49, 0x49, 0x49, 0x36 }, /* 0x42  'B' */
    { 0x3E, 0x41, 0x41, 0x41, 0x22 }, /* 0x43  'C' */
    { 0x7F, 0x41, 0x41, 0x22, 0x1C }, /* 0x44  'D' */
    { 0x7F, 0x49, 0x49, 0x49, 0x41 }, /* 0x45  'E' */
    { 0x7F, 0x09, 0x09, 0x09, 0x01 }, /* 0x46  'F' */
    { 0x3E, 0x41, 0x49, 0x49, 0x7A }, /* 0x47  'G' */
    { 0x7F, 0x08, 0x08, 0x08, 0x7F }, /* 0x48  'H' */
    { 0x00, 0x41, 0x7F, 0x41, 0x00 }, /* 0x49  'I' */
    { 0x20, 0x40, 0x41, 0x3F, 0x01 }, /* 0x4A  'J' */
    { 0x7F, 0x08, 0x14, 0x22, 0x41 }, /* 0x4B  'K' */
    { 0x7F, 0x40, 0x40, 0x40, 0x40 }, /* 0x4C  'L' */
    { 0x7F, 0x02, 0x04, 0x02, 0x7F }, /* 0x4D  'M' */
    { 0x7F, 0x04, 0x08, 0x10, 0x7F }, /* 0x4E  'N' */
    { 0x3E, 0x41, 0x41, 0x41, 0x3E }, /* 0x4F  'O' */
    { 0x7F, 0x09, 0x09, 0x09, 0x06 }, /* 0x50  'P' */
    { 0x3E, 0x41, 0x51, 0x21, 0x5E }, /* 0x51  'Q' */
    { 0x7F, 0x09, 0x19, 0x29, 0x46 }, /* 0x52  'R' */
    { 0x46, 0x49, 0x49, 0x49, 0x31 }, /* 0x53  'S' */
    { 0x01, 0x01, 0x7F, 0x01, 0x01 }, /* 0x54  'T' */
    { 0x3F, 0x40, 0x40, 0x40, 0x3F }, /* 0x55  'U' */
    { 0x1F, 0x20, 0x40, 0x20, 0x1F }, /* 0x56  'V' */
    { 0x3F, 0x40, 0x38, 0x40, 0x3F }, /* 0x57  'W' */
    { 0x63, 0x14, 0x08, 0x14, 0x63 }, /* 0x58  'X' */
    { 0x07, 0x08, 0x70, 0x08, 0x07 }, /* 0x59  'Y' */
    { 0x61, 0x51, 0x49, 0x45, 0x43 }, /* 0x5A  'Z' */
    { 0x00, 0x7F, 0x41, 0x41, 0x00 }, /* 0x5B  '[' */
    { 0x02, 0x04, 0x08, 0x10, 0x20 }, /* 0x5C  '\' */
    { 0x00, 0x41, 0x41, 0x7F, 0x00 }, /* 0x5D  ']' */
    { 0x04, 0x02, 0x01, 0x02, 0x04 }, /* 0x5E  '^' */
    { 0x40, 0x40, 0x40, 0x40, 0x40 }, /* 0x5F  '_' */
    { 0x00, 0x01, 0x02, 0x04, 0x00 }, /* 0x60  '`' */
    { 0x20, 0x54, 0x54, 0x54, 0x78 }, /* 0x61  'a' */
    { 0x7F, 0x48, 0x44, 0x44, 0x38 }, /* 0x62  'b' */
    { 0x38, 0x44, 0x44, 0x44, 0x20 }, /* 0x63  'c' */
    { 0x38, 0x44, 0x44, 0x48, 0x7F }, /* 0x64  'd' */
    { 0x38, 0x54, 0x54, 0x54, 0x18 }, /* 0x65  'e' */
    { 0x08, 0x7E, 0x09, 0x01, 0x02 }, /* 0x66  'f' */
    { 0x0C, 0x52, 0x52, 0x52, 0x3E }, /* 0x67  'g' */
    { 0x7F, 0x08, 0x04, 0x04, 0x78 }, /* 0x68  'h' */
    { 0x00, 0x44, 0x7D, 0x40, 0x00 }, /* 0x69  'i' */
    { 0x20, 0x40, 0x44, 0x3D, 0x00 }, /* 0x6A  'j' */
    { 0x7F, 0x10, 0x28, 0x44, 0x00 }, /* 0x6B  'k' */
    { 0x00, 0x41, 0x7F, 0x40, 0x00 }, /* 0x6C  'l' */
    { 0x7C, 0x04, 0x18, 0x04, 0x78 }, /* 0x6D  'm' */
    { 0x7C, 0x08, 0x04, 0x04, 0x78 }, /* 0x6E  'n' */
    { 0x38, 0x44, 0x44, 0x44, 0x38 }, /* 0x6F  'o' */
    { 0x7C, 0x14, 0x14, 0x14, 0x08 }, /* 0x70  'p' */
    { 0x08, 0x14, 0x14, 0x18, 0x7C }, /* 0x71  'q' */
    { 0x7C, 0x08, 0x04, 0x04, 0x08 }, /* 0x72  'r' */
    { 0x48, 0x54, 0x54, 0x54, 0x20 }, /* 0x73  's' */
    { 0x04, 0x3F, 0x44, 0x40, 0x20 }, /* 0x74  't' */
    { 0x3C, 0x40, 0x40, 0x20, 0x7C }, /* 0x75  'u' */
    { 0x1C, 0x20, 0x40, 0x20, 0x1C }, /* 0x76  'v' */
    { 0x3C, 0x40, 0x30, 0x40, 0x3C }, /* 0x77  'w' */
    { 0x44, 0x28, 0x10, 0x28, 0x44 }, /* 0x78  'x' */
    { 0x0C, 0x50, 0x50, 0x50, 0x3C }, /* 0x79  'y' */
    { 0x44, 0x64, 0x54, 0x4C, 0x44 }, /* 0x7A  'z' */
    { 0x00, 0x08, 0x36, 0x41, 0x00 }, /* 0x7B  '{' */
    { 0x00, 0x00, 0x7F, 0x00, 0x00 }, /* 0x7C  '|' */
    { 0x00, 0x41, 0x36, 0x08, 0x00 }, /* 0x7D  '}' */
    { 0x10, 0x08, 0x08, 0x10, 0x08 }, /* 0x7E  '~' */
};

/* =========================================================
 * Fonctions statiques internes
 * ========================================================= */

/**
 * @brief Envoie une commande SSD1306 via I2C
 */

static HAL_StatusTypeDef SSD1306_WriteCmd(I2C_HandleTypeDef *hi2c, uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};

    HAL_StatusTypeDef status =
        HAL_I2C_Master_Transmit(
            hi2c,
            SSD1306_I2C_ADDR << 1,
            buf,
            2,
            100);

    return status;
}

/* =========================================================
 * Fonctions publiques
 * ========================================================= */

HAL_StatusTypeDef SSD1306_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_Delay(10);

    const uint8_t init_cmds[] = {
        SSD1306_DISPLAYOFF,
        SSD1306_SETDISPLAYCLOCKDIV,
        0x80,
        SSD1306_SETMULTIPLEX,
        0x3F,
        SSD1306_SETDISPLAYOFFSET,
        0x00,
        SSD1306_SETSTARTLINE | 0x00,
        SSD1306_CHARGEPUMP,
        0x14,
        SSD1306_MEMORYMODE,
        0x00,
        SSD1306_SEGREMAP | 0x01,
        SSD1306_COMSCANDEC,
        SSD1306_SETCOMPINS,
        0x12,
        SSD1306_SETCONTRAST,
        0xCF,
        SSD1306_SETPRECHARGE,
        0xF1,
        SSD1306_SETVCOMDETECT,
        0x40,
        SSD1306_DISPLAYALLON_RESUME,
        SSD1306_NORMALDISPLAY,
        SSD1306_DISPLAYON,
    };

    // Envoyer la séquence d'initialisation au SSD1306 via I2C
    for (uint8_t i = 0; i < sizeof(init_cmds); i++)
    {
        HAL_StatusTypeDef status;
        status = SSD1306_WriteCmd(hi2c, init_cmds[i]);
        if(status != HAL_OK)
        {
            return (HAL_StatusTypeDef)(0x10 + i);
        }
    } // Fin du for

    SSD1306_ClearBuffer();
    SSD1306_Display(hi2c);

    return HAL_OK;
}


void SSD1306_ClearBuffer(void)
{
    memset(ssd1306_buffer, 0x00, sizeof(ssd1306_buffer));
}

/* =========================================================
 * Fonctions de dessin
 * =========================================================
 */
void SSD1306_Display(I2C_HandleTypeDef *hi2c)
{
    /* Positionner au début : page 0, colonne 0 */
    SSD1306_WriteCmd(hi2c, SSD1306_COLUMNADDR);
    SSD1306_WriteCmd(hi2c, 0);    /* colonne début */
    SSD1306_WriteCmd(hi2c, SSD1306_WIDTH - 1); /* colonne fin */

    SSD1306_WriteCmd(hi2c, SSD1306_PAGEADDR);
    SSD1306_WriteCmd(hi2c, 0);    /* page début */
    SSD1306_WriteCmd(hi2c, (SSD1306_HEIGHT / 8) - 1); /* page fin */

    /* Envoyer les données en blocs de 16 octets
     * Protocole I2C SSD1306 : préfixe 0x40 = Co=0, D/C#=1 → données */
    uint8_t buf[17];
    buf[0] = 0x40;

    for (uint16_t i = 0; i < sizeof(ssd1306_buffer); i += 16) {
        uint8_t chunk = 16;
        if ((uint16_t)(i + 16) > sizeof(ssd1306_buffer)) {
            chunk = (uint8_t)(sizeof(ssd1306_buffer) - i);
        }
        memcpy(&buf[1], &ssd1306_buffer[i], chunk);
        HAL_I2C_Master_Transmit(hi2c, SSD1306_I2C_ADDR << 1, buf, chunk + 1, HAL_MAX_DELAY);
    }
}

/* =========================================================
 * Fonctions de dessin de pixels et caractères
 * =========================================================
 */
void SSD1306_DrawPixel(int16_t x, int16_t y, uint8_t color)
{
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) {
        return;
    }
    if (color) {
        ssd1306_buffer[x + (y / 8) * SSD1306_WIDTH] |=  (uint8_t)(1u << (y & 7));
    } else {
        ssd1306_buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(uint8_t)(1u << (y & 7));
    }
}

/* =========================================================
 * Fonctions de dessin de caractères et chaînes
 * =========================================================
 */
void SSD1306_DrawChar(int16_t x, int16_t y, char c, uint8_t size, uint8_t color)
{
    if (c < 0x20 || c > 0x7E) c = '?';

    const uint8_t *glyph = font5x7[(uint8_t)(c - 0x20)];

    for (int8_t col = 0; col < 5; col++) {
        uint8_t line = glyph[col];
        for (int8_t row = 0; row < 7; row++) {
            if (line & (1u << row)) {
                if (size == 1) {
                    SSD1306_DrawPixel(x + col, y + row, color);
                } else {
                    SSD1306_FillRect(x + col * size, y + row * size, size, size, color);
                }
            }
        }
    }
}

/**
 * @brief Dessine une chaîne de caractères sur l'écran
 * @param x : position horizontale (0 à 127)
 * @param y : position verticale (0 à 63)
 * @param str : chaîne de caractères à afficher
 * @param size : taille du texte (1 = normal, 2 = double, etc.)
 * @param color : couleur du texte (SSD1306_WHITE ou SSD1306_BLACK)
 */
void SSD1306_DrawString(int16_t x, int16_t y, const char *str, uint8_t size, uint8_t color)
{
    int16_t cx = x;
    while (*str) {
        if (*str == '\n') {
            y += (int16_t)(FONT_HEIGHT * size);
            cx = x;
        } else {
            SSD1306_DrawChar(cx, y, *str, size, color);
            cx += (int16_t)(FONT_WIDTH * size);
        }
        str++;
    }
}

/**
 * @brief Remplit un rectangle sur l'écran
 * @param x : position horizontale du coin supérieur gauche
 * @param y : position verticale du coin supérieur gauche
 * @param w : largeur du rectangle
 * @param h : hauteur du rectangle
 * @param color : couleur du rectangle (SSD1306_WHITE ou SSD1306_BLACK)
 */
void SSD1306_FillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color)
{
    for (int16_t i = x; i < x + w; i++) {
        for (int16_t j = y; j < y + h; j++) {
            SSD1306_DrawPixel(i, j, color);
        }
    }
}

/**
 * @brief Dessine un rectangle vide sur l'écran
 * @param x : position horizontale du coin supérieur gauche
 * @param y : position verticale du coin supérieur gauche
 * @param w : largeur du rectangle
 * @param h : hauteur du rectangle
 * @param color : couleur du rectangle (SSD1306_WHITE ou SSD1306_BLACK)
 */
void SSD1306_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color)
{
    /* Côté supérieur */
    for (int16_t i = x; i < x + w; i++) SSD1306_DrawPixel(i, y, color);
    /* Côté inférieur */
    for (int16_t i = x; i < x + w; i++) SSD1306_DrawPixel(i, y + h - 1, color);
    /* Côté gauche */
    for (int16_t j = y; j < y + h; j++) SSD1306_DrawPixel(x, j, color);
    /* Côté droit */
    for (int16_t j = y; j < y + h; j++) SSD1306_DrawPixel(x + w - 1, j, color);
}
// Cleaned up stray/unmatched closing braces here