/**************************************************************************
 *  @file     pn532.c
 *  @author   Yehui from Waveshare
 *  @license  BSD
 *
 *  This is a library for the Waveshare PN532 NFC modules
 *
 *  Check out the links above for our tutorials and wiring diagrams
 *  These chips use SPI communicate.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documnetation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to  whom the Software is
 * furished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 **************************************************************************/

#include "PN532.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


const uint8_t PN532_ACK[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
const uint8_t PN532_FRAME_START[] = {0x00, 0x00, 0xFF};

#define PN532_FRAME_MAX_LENGTH              255
#define PN532_DEFAULT_TIMEOUT               1000

/**
  * @brief: Write a frame to the PN532 of at most length bytes in size.
  *     Note that less than length bytes might be returned!
  * @retval: Returns -1 if there is an error parsing the frame.
  */
int PN532_WriteFrame(PN532* pn532, uint8_t* data, uint16_t length) {
    if (length > PN532_FRAME_MAX_LENGTH || length < 1) {
        return PN532_STATUS_ERROR; // Data must be array of 1 to 255 bytes.
    }
    // Build frame to send as:
    // - Preamble (0x00)
    // - Start code  (0x00, 0xFF)
    // - Command length (1 byte)
    // - Command length checksum
    // - Command bytes
    // - Checksum
    // - Postamble (0x00)

    uint8_t frame[PN532_FRAME_MAX_LENGTH + 7];
    uint8_t checksum = 0;
    frame[0] = PN532_PREAMBLE;
    frame[1] = PN532_STARTCODE1;
    frame[2] = PN532_STARTCODE2;
    for (uint8_t i = 0; i < 3; i++) {
        checksum += frame[i];
    }
    frame[3] = length & 0xFF;
    frame[4] = (~length + 1) & 0xFF;
    for (uint8_t i = 0; i < length; i++) {
        frame[5 + i] = data[i];
        checksum += data[i];
    }
    frame[length + 5] = ~checksum & 0xFF;
    frame[length + 6] = PN532_POSTAMBLE;
     if (pn532->write_data(frame, length + 7) != PN532_STATUS_OK) {
        return PN532_STATUS_ERROR;
    }
    return PN532_STATUS_OK;
}

/**
  * @brief: Read a response frame from the PN532 of at most length bytes in size.
  *     Note that less than length bytes might be returned!
  * @retval: Returns frame length or -1 if there is an error parsing the frame.
  */
int PN532_ReadFrame(PN532* pn532, uint8_t* response, uint16_t length) {
    uint8_t buff[PN532_FRAME_MAX_LENGTH + 7];
    uint8_t checksum = 0;
    // Read frame with expected length of data.
    pn532->read_data(buff, length + 7);
    // Swallow all the 0x00 values that preceed 0xFF.
    uint8_t offset = 0;
    while (buff[offset] == 0x00) {
        offset += 1;
        if (offset >= length + 8){
            pn532->log("Response frame preamble does not contain 0x00FF!");
            return PN532_STATUS_ERROR;
        }
    }
    if (buff[offset] != 0xFF) {
        pn532->log("Response frame preamble does not contain 0x00FF!");
        return PN532_STATUS_ERROR;
    }
    offset += 1;
    if (offset >= length + 8) {
        pn532->log("Response contains no data!");
        return PN532_STATUS_ERROR;
    }
    // Check length & length checksum match.
    uint8_t frame_len = buff[offset];
    if (((frame_len + buff[offset+1]) & 0xFF) != 0) {
        pn532->log("Response length checksum did not match length!");
        return PN532_STATUS_ERROR;
    }
    // Check frame checksum value matches bytes.
    for (uint8_t i = 0; i < frame_len + 1; i++) {
        checksum += buff[offset + 2 + i];
    }
    checksum &= 0xFF;
    if (checksum != 0) {
        pn532->log("Response checksum did not match expected checksum");
        return PN532_STATUS_ERROR;
    }
    // Return frame data.
    for (uint8_t i = 0; i < frame_len; i++) {
        response[i] = buff[offset + 2 + i];
    }
    return frame_len;
}

/**
  * @brief: Send specified command to the PN532 and expect up to response_length.
  *     Will wait up to timeout seconds for a response and read a bytearray into
  *     response buffer.
  * @param pn532: PN532 handler
  * @param command: command to send
  * @param response: buffer returned
  * @param response_length: expected response length
  * @param params: can optionally specify an array of bytes to send as parameters
  *     to the function call, or NULL if there is no need to send parameters.
  * @param params_length: length of the argument params
  * @param timeout: timout of systick
  * @retval: Returns the length of response or -1 if error.
  */
int PN532_CallFunction(
    PN532* pn532,
    uint8_t command,
    uint8_t* response,
    uint16_t response_length,
    uint8_t* params,
    uint16_t params_length,
    uint32_t timeout
) {
    // Build frame data with command and parameters.
    uint8_t buff[PN532_FRAME_MAX_LENGTH];
    buff[0] = PN532_HOSTTOPN532;
    buff[1] = command & 0xFF;
    for (uint8_t i = 0; i < params_length; i++) {
        buff[2 + i] = params[i];
    }
    // Send frame and wait for response.
    if (PN532_WriteFrame(pn532, buff, params_length + 2) != PN532_STATUS_OK) {
        pn532->wakeup();
        pn532->log("Trying to wakeup");
        return PN532_STATUS_ERROR;
    }
    if (!pn532->wait_ready(timeout)) {
        return PN532_STATUS_ERROR;
    }
    // Verify ACK response and wait to be ready for function response.
    pn532->read_data(buff, sizeof(PN532_ACK));
    for (uint8_t i = 0; i < sizeof(PN532_ACK); i++) {
        if (PN532_ACK[i] != buff[i]) {
            pn532->log("Did not receive expected ACK from PN532!");
            return PN532_STATUS_ERROR;
        }
    }
    if (!pn532->wait_ready(timeout)) {
        return PN532_STATUS_ERROR;
    }
    // Read response bytes.
    int frame_len = PN532_ReadFrame(pn532, buff, response_length + 2);

    // Check that response is for the called function.
    if (! ((buff[0] == PN532_PN532TOHOST) && (buff[1] == (command+1)))) {
        pn532->log("Received unexpected command response!");
        return PN532_STATUS_ERROR;
    }
    // Return response data.
    for (uint8_t i = 0; i < response_length; i++) {
        response[i] = buff[i + 2];
    }
    // The the number of bytes read
    return frame_len - 2;
}

/**
  * @brief: Call PN532 GetFirmwareVersion function and return a buff with the IC,
  *  Ver, Rev, and Support values.
  */
int PN532_GetFirmwareVersion(PN532* pn532, uint8_t* version) {
    // length of version: 4
    if (PN532_CallFunction(pn532, PN532_COMMAND_GETFIRMWAREVERSION,
                           version, 4, NULL, 0, 500) == PN532_STATUS_ERROR) {
        pn532->log("Failed to detect the PN532");
        return PN532_STATUS_ERROR;
    }
    return PN532_STATUS_OK;
}

/**
  * @brief: Configure the PN532 to read MiFare cards.
  */
int PN532_SamConfiguration(PN532* pn532) {
    // Send SAM configuration command with configuration for:
    // - 0x01, normal mode
    // - 0x14, timeout 50ms * 20 = 1 second
    // - 0x01, use IRQ pin
    // Note that no other verification is necessary as call_function will
    // check the command was executed as expected.
    //uint8_t params[] = {0x01, 0x14, 0x01};
    uint8_t params[] = {0x4A, 0x01, 0x00};
    PN532_CallFunction(pn532, PN532_COMMAND_SAMCONFIGURATION,
                       NULL, 0, params, sizeof(params), PN532_DEFAULT_TIMEOUT);
    return PN532_STATUS_OK;
}

/**
  * @brief: Wait for a MiFare card to be available and return its UID when found.
  *     Will wait up to timeout seconds and return None if no card is found,
  *     otherwise a bytearray with the UID of the found card is returned.
  * @retval: Length of UID, or -1 if error.
  */
int PN532_ReadPassiveTarget(
    PN532* pn532,
    uint8_t* response,
    uint8_t card_baud,
    uint32_t timeout
) {
    // Send passive read command for 1 card.  Expect at most a 7 byte UUID.
    uint8_t params[] = {0x01, card_baud};
    uint8_t buff[19];
    int length = PN532_CallFunction(pn532, PN532_COMMAND_INLISTPASSIVETARGET,
                        buff, sizeof(buff), params, sizeof(params), timeout);
    if (length < 0) {
        return PN532_STATUS_ERROR; // No card found
    }
    // Check only 1 card with up to a 7 byte UID is present.
    if (buff[0] != 0x01) {
        pn532->log("More than one card detected!");
        return PN532_STATUS_ERROR;
    }
    if (buff[5] > 7) {
        pn532->log("Found card with unexpectedly long UID!");
        return PN532_STATUS_ERROR;
    }
    for (uint8_t i = 0; i < buff[5]; i++) {
        response[i] = buff[6 + i];
    }
    return buff[5];
}

/**
  * @brief: Authenticate specified block number for a MiFare classic card.
  * @param uid: A byte array with the UID of the card.
  * @param uid_length: Length of the UID of the card.
  * @param block_number: The block to authenticate.
  * @param key_number: The key type (like MIFARE_CMD_AUTH_A or MIFARE_CMD_AUTH_B).
  * @param key: A byte array with the key data.
  * @retval: PN532 error code.
  */
int PN532_MifareClassicAuthenticateBlock(
    PN532* pn532,
    uint8_t* uid,
    uint8_t uid_length,
    uint16_t block_number,
    uint16_t key_number,
    uint8_t* key
) {
    // Build parameters for InDataExchange command to authenticate MiFare card.
    uint8_t response[1] = {0xFF};
    uint8_t params[3 + MIFARE_UID_MAX_LENGTH + MIFARE_KEY_LENGTH];
    params[0] = 0x01;
    params[1] = key_number & 0xFF;
    params[2] = block_number & 0xFF;
    // params[3:3+keylen] = key
    for (uint8_t i = 0; i < MIFARE_KEY_LENGTH; i++) {
        params[3 + i] = key[i];
    }
    // params[3+keylen:] = uid
    for (uint8_t i = 0; i < uid_length; i++) {
        params[3 + MIFARE_KEY_LENGTH + i] = uid[i];
    }
    // Send InDataExchange request
    PN532_CallFunction(pn532, PN532_COMMAND_INDATAEXCHANGE, response, sizeof(response),
                       params, 3 + MIFARE_KEY_LENGTH + uid_length, PN532_DEFAULT_TIMEOUT);
    return response[0];
}

/**
  * @brief: Read a block of data from the card. Block number should be the block
  *     to read.
  * @param response: buffer of length 16 returned if the block is successfully read.
  * @param block_number: specify a block to read.
  * @retval: PN532 error code.
  */
int PN532_MifareClassicReadBlock(PN532* pn532, uint8_t* response, uint16_t block_number) {
    uint8_t params[] = {0x01, MIFARE_CMD_READ, block_number & 0xFF};
    uint8_t buff[MIFARE_BLOCK_LENGTH + 1];
    // Send InDataExchange request to read block of MiFare data.
    PN532_CallFunction(pn532, PN532_COMMAND_INDATAEXCHANGE, buff, sizeof(buff),
                       params, sizeof(params), PN532_DEFAULT_TIMEOUT);
    // Check first response is 0x00 to show success.
    if (buff[0] != PN532_ERROR_NONE) {
        return buff[0];
    }
    for (uint8_t i = 0; i < MIFARE_BLOCK_LENGTH; i++) {
        response[i] = buff[i + 1];
    }
    return buff[0];
}

/**
  * @brief: Write a block of data to the card.  Block number should be the block
  *     to write and data should be a byte array of length 16 with the data to
  *     write.
  * @param data: data to write.
  * @param block_number: specify a block to write.
  * @retval: PN532 error code.
  */
int PN532_MifareClassicWriteBlock(PN532* pn532, uint8_t* data, uint16_t block_number) {
    uint8_t params[MIFARE_BLOCK_LENGTH + 3];
    uint8_t response[1];
    params[0] = 0x01;  // Max card numbers
    params[1] = MIFARE_CMD_WRITE;
    params[2] = block_number & 0xFF;
    for (uint8_t i = 0; i < MIFARE_BLOCK_LENGTH; i++) {
        params[3 + i] = data[i];
    }
    PN532_CallFunction(pn532, PN532_COMMAND_INDATAEXCHANGE, response,
                       sizeof(response), params, sizeof(params), PN532_DEFAULT_TIMEOUT);
    return response[0];
}

/**
  * @brief: Read a block of data from the card. Block number should be the block
  *     to read.
  * @param response: buffer of length 4 returned if the block is successfully read.
  * @param block_number: specify a block to read.
  * @retval: PN532 error code.
  */
int PN532_Ntag2xxReadBlock(PN532* pn532, uint8_t* response, uint16_t block_number) {
    uint8_t params[] = {0x01, MIFARE_CMD_READ, block_number & 0xFF};
    // The response length of NTAG2xx is same as Mifare's
    uint8_t buff[MIFARE_BLOCK_LENGTH + 1];
    // Send InDataExchange request to read block of MiFare data.
    PN532_CallFunction(pn532, PN532_COMMAND_INDATAEXCHANGE, buff, sizeof(buff),
                       params, sizeof(params), PN532_DEFAULT_TIMEOUT);
    // Check first response is 0x00 to show success.
    if (buff[0] != PN532_ERROR_NONE) {
        return buff[0];
    }
    // Although the response length of NTAG2xx is same as Mifare's,
    // only the first 4 bytes are available
    for (uint8_t i = 0; i < NTAG2XX_BLOCK_LENGTH; i++) {
        response[i] = buff[i + 1];
    }
    return buff[0];
}

/**
  * @brief: Write a block of data to the card.  Block number should be the block
  *     to write and data should be a byte array of length 4 with the data to
  *     write.
  * @param data: data to write.
  * @param block_number: specify a block to write.
  * @retval: PN532 error code.
  */
int PN532_Ntag2xxWriteBlock(PN532* pn532, uint8_t* data, uint16_t block_number) {
    uint8_t params[NTAG2XX_BLOCK_LENGTH + 3];
    uint8_t response[1];
    params[0] = 0x01;  // Max card numbers
    params[1] = MIFARE_ULTRALIGHT_CMD_WRITE;
    params[2] = block_number & 0xFF;
    for (uint8_t i = 0; i < NTAG2XX_BLOCK_LENGTH; i++) {
        params[3 + i] = data[i];
    }
    PN532_CallFunction(pn532, PN532_COMMAND_INDATAEXCHANGE, response,
                       sizeof(response), params, sizeof(params), PN532_DEFAULT_TIMEOUT);
    return response[0];
}

/**
  * @brief: Read the GPIO states.
  * @param pin_state: pin state buffer (3 bytes) returned.
  * returns 3 bytes containing the pin state where:
  *     P3[0] = P30,   P7[0] = 0,   I[0] = I0,
  *     P3[1] = P31,   P7[1] = P71, I[1] = I1,
  *     P3[2] = P32,   P7[2] = P72, I[2] = 0,
  *     P3[3] = P33,   P7[3] = 0,   I[3] = 0,
  *     P3[4] = P34,   P7[4] = 0,   I[4] = 0,
  *     P3[5] = P35,   P7[5] = 0,   I[5] = 0,
  *     P3[6] = 0,     P7[6] = 0,   I[6] = 0,
  *     P3[7] = 0,     P7[7] = 0,   I[7] = 0,
  * @retval: -1 if error
  */
int PN532_ReadGpio(PN532* pn532, uint8_t* pins_state) {
    return PN532_CallFunction(pn532, PN532_COMMAND_READGPIO, pins_state, 3,
                              NULL, 0, PN532_DEFAULT_TIMEOUT);
}
/**
  * @brief: Read the GPIO state of specified pins in (P30 ... P35).
  * @param pin_number: specify the pin to read.
  * @retval: true if HIGH, false if LOW
  */
bool PN532_ReadGpioP(PN532* pn532, uint8_t pin_number) {
    uint8_t pins_state[3];
    PN532_CallFunction(pn532, PN532_COMMAND_READGPIO, pins_state,
                       sizeof(pins_state), NULL, 0, PN532_DEFAULT_TIMEOUT);
    if ((pin_number >= 30) && (pin_number <= 37)) {
        return (pins_state[0] >> (pin_number - 30)) & 1 ? true : false;
    }
    if ((pin_number >= 70) && (pin_number <= 77)) {
        return (pins_state[1] >> (pin_number - 70)) & 1 ? true : false;
    }
    return false;
}
/**
  * @brief: Read the GPIO state of I0 or I1 pin.
  * @param pin_number: specify the pin to read.
  * @retval: true if HIGH, false if LOW
  */
bool PN532_ReadGpioI(PN532* pn532, uint8_t pin_number) {
    uint8_t pins_state[3];
    PN532_CallFunction(pn532, PN532_COMMAND_READGPIO, pins_state,
                       sizeof(pins_state), NULL, 0, PN532_DEFAULT_TIMEOUT);
    if (pin_number <= 7) {
        return (pins_state[2] >> pin_number) & 1 ? true : false;
    }
    return false;
}
/**
  * @brief: Write the GPIO states.
  * @param pins_state: pin state buffer (2 bytes) to write.
  *     no need to read pin states before write with the param pin_state
  *         P3 = pin_state[0], P7 = pin_state[1]
  *     bits:
  *         P3[0] = P30,   P7[0] = 0,
  *         P3[1] = P31,   P7[1] = P71,
  *         P3[2] = P32,   P7[2] = P72,
  *         P3[3] = P33,   P7[3] = nu,
  *         P3[4] = P34,   P7[4] = nu,
  *         P3[5] = P35,   P7[5] = nu,
  *         P3[6] = nu,    P7[6] = nu,
  *         P3[7] = Val,   P7[7] = Val,
  *     For each port that is validated (bit Val = 1), all the bits are applied
  *     simultaneously. It is not possible for example to modify the state of
  *     the port P32 without applying a value to the ports P30, P31, P33, P34
  *     and P35.
  * @retval: -1 if error
  */
int PN532_WriteGpio(PN532* pn532, uint8_t* pins_state) {
    uint8_t params[2];
    // 0x80, the validation bit.
    params[0] = 0x80 | pins_state[0];
    params[1] = 0x80 | pins_state[1];
    return PN532_CallFunction(pn532, PN532_COMMAND_WRITEGPIO, NULL, 0,
                              params, sizeof(params), PN532_DEFAULT_TIMEOUT);
}
/**
  * @brief: Write the specified pin with given states.
  * @param pin_number: specify the pin to write.
  * @param pin_state: specify the pin state. true for HIGH, false for LOW.
  * @retval: -1 if error
  */
int PN532_WriteGpioP(PN532* pn532, uint8_t pin_number, bool pin_state) {
    uint8_t pins_state[2];
    uint8_t params[2];
    if (PN532_ReadGpio(pn532, pins_state) == PN532_STATUS_ERROR) {
        return PN532_STATUS_ERROR;
    }
    if ((pin_number >= 30) && (pin_number <= 37)) {
        if (pin_state) {
            params[0] = 0x80 | pins_state[0] | 1 << (pin_number - 30);
        } else {
            params[0] = (0x80 | pins_state[0]) & ~(1 << (pin_number - 30));
        }
        params[1] = 0x00;   // leave p7 unchanged
    }
    if ((pin_number >= 70) && (pin_number <= 77)) {
        if (pin_state) {
            params[1] = 0x80 | pins_state[1] | 1 << (pin_number - 70);
        } else {
            params[1] = (0x80 | pins_state[1]) & ~(1 << (pin_number - 70));
        }
        params[0] = 0x00;   // leave p3 unchanged
    }
    return PN532_CallFunction(pn532, PN532_COMMAND_WRITEGPIO, NULL, 0,
                              params, sizeof(params), PN532_DEFAULT_TIMEOUT);
}

/****************************CIN-PN532****************************************/
/****************************CIN-PN532****************************************/

// Définition des constantes
#define MAX_FRAME_SIZE 256
#define KEY_SIZE 16
#define NONCE_SIZE 8
#define MRZ_SIZE 24

// Structure pour stocker les clés BAC
typedef struct {
    uint8_t Ksenc[16];  // Clé de session pour le chiffrement
    uint8_t Ksmac[16];  // Clé de session pour le MAC
} BACKeys;

// Variables globales
static uint8_t mrz_key[KEY_SIZE];
static BACKeys session_keys;
static I2C_HandleTypeDef hi2c1;

// Fonctions utilitaires pour le protocole BAC
static void calculate_sha1(const uint8_t* data, size_t length, uint8_t* hash) {
    // Implémentation de SHA-1 (utiliser une bibliothèque crypto comme mbedtls)
    mbedtls_sha1_context ctx;
    mbedtls_sha1_init(&ctx);
    mbedtls_sha1_starts(&ctx);
    mbedtls_sha1_update(&ctx, data, length);
    mbedtls_sha1_finish(&ctx, hash);
    mbedtls_sha1_free(&ctx);
}

static void calculate_3des(uint8_t* input, uint8_t* output, uint8_t* key, int mode) {
    // Implémentation du 3DES (utiliser une bibliothèque crypto comme mbedtls)
    mbedtls_des3_context ctx;
    mbedtls_des3_init(&ctx);

    if (mode == MBEDTLS_DES_ENCRYPT) {
        mbedtls_des3_set3key_enc(&ctx, key);
        mbedtls_des3_crypt_ecb(&ctx, input, output);
    } else {
        mbedtls_des3_set3key_dec(&ctx, key);
        mbedtls_des3_crypt_ecb(&ctx, input, output);
    }

    mbedtls_des3_free(&ctx);
}

// Fonction pour calculer la clé MRZ
static void calculate_mrz_key(const char* CIN_num, const char* birth_date, const char* expiry_date) {
    uint8_t mrz_info[MRZ_SIZE];
    uint8_t hash[20];

    // Concaténer les informations MRZ
    memcpy(mrz_info, CIN_num, 7);
    memcpy(mrz_info + 7, birth_date, 6);
    memcpy(mrz_info + 13, expiry_date, 6);

    // Calculer le SHA-1
    calculate_sha1(mrz_info, MRZ_SIZE, hash);

    // Prendre les 16 premiers octets pour la clé
    memcpy(mrz_key, hash, KEY_SIZE);
}

// Fonction pour dériver les clés de session
static void derive_session_keys(uint8_t* k_seed) {
    uint8_t hash[20];
    uint8_t enc_data[20] = {0};
    uint8_t mac_data[20] = {0};

    // Dérivation pour Ksenc
    enc_data[0] = 0x00;
    enc_data[1] = 0x00;
    enc_data[2] = 0x00;
    enc_data[3] = 0x01;
    memcpy(enc_data + 4, k_seed, 16);
    calculate_sha1(enc_data, 20, hash);
    memcpy(session_keys.Ksenc, hash, 16);

    // Dérivation pour Ksmac
    mac_data[0] = 0x00;
    mac_data[1] = 0x00;
    mac_data[2] = 0x00;
    mac_data[3] = 0x02;
    memcpy(mac_data + 4, k_seed, 16);
    calculate_sha1(mac_data, 20, hash);
    memcpy(session_keys.Ksmac, hash, 16);
}

// Fonction principale pour l'authentification BAC
int perform_bac(PN532* pn532) {
    uint8_t rnd_icc[8], rnd_ifd[8], k_ifd[16];
    uint8_t encrypted_challenge[32];
    uint8_t response[MAX_FRAME_SIZE];
    uint8_t cmd_data[40];

    // 1. Génération du challenge aléatoire
    for(int i = 0; i < 8; i++) {
        rnd_ifd[i] = rand() % 256;
    }

    // 2. Envoi de GET CHALLENGE
    uint8_t get_challenge[] = {0x00, 0x84, 0x00, 0x00, 0x08};
    if (PN532_CallFunction(&pn532, PN532_COMMAND_INDATAEXCHANGE, response, sizeof(response), get_challenge, sizeof(get_challenge), PN532_DEFAULT_TIMEOUT) <= 0) {
        return 0;
    }
    memcpy(rnd_icc, response, 8);

    // 3. Construction de la commande EXTERNAL AUTHENTICATE
    memcpy(cmd_data, rnd_ifd, 8);
    memcpy(cmd_data + 8, rnd_icc, 8);
    memcpy(cmd_data + 16, k_ifd, 16);

    // 4. Chiffrement du challenge
    calculate_3des(cmd_data, encrypted_challenge, mrz_key, MBEDTLS_DES_ENCRYPT);

    // 5. Envoi de la commande d'authentification
    uint8_t auth_cmd[40] = {0x00, 0x82, 0x00, 0x00, 0x28};
    memcpy(auth_cmd + 5, encrypted_challenge, 32);
    if (PN532_CallFunction(&pn532, PN532_COMMAND_INDATAEXCHANGE, response, sizeof(response), auth_cmd, sizeof(auth_cmd), PN532_DEFAULT_TIMEOUT) <= 0) {
        return 0;
    }

    // 6. Dérivation des clés de session
    uint8_t k_seed[32];
    memcpy(k_seed, rnd_ifd, 8);
    memcpy(k_seed + 8, rnd_icc, 8);
    memcpy(k_seed + 16, k_ifd, 16);
    derive_session_keys(k_seed);

    return 1;
}

// Fonction pour lire les données après authentification BAC
int CIN_data(PN532* pn532, uint8_t* data_buffer, size_t* data_length) {
    uint8_t cmd[5] = {0x00, 0xB0, 0x00, 0x00, 0x00};  // READ BINARY
    uint8_t response[MAX_FRAME_SIZE];
    int resp_len;

    // Protection des commandes avec Secure Messaging
    uint8_t protected_cmd[MAX_FRAME_SIZE];
    size_t protected_len;

    // Application du Secure Messaging
    /*if (!apply_secure_messaging(cmd, sizeof(cmd), protected_cmd, &protected_len)) {
        return 0;
    }*/

    // Envoi de la commande protégée
    resp_len = PN532_CallFunction(&pn532, PN532_COMMAND_INDATAEXCHANGE, response, sizeof(response), protected_cmd, protected_len, PN532_DEFAULT_TIMEOUT);
    if (resp_len <= 0) {
        return 0;
    }

    // Déchiffrement de la réponse
    /*if (!decrypt_secure_messaging(response, resp_len, data_buffer, data_length)) {
        return 0;
    }*/

    return 1;
}

// Fonction principale pour initialiser et utiliser le système
int read_identity_card(PN532* pn532, const char* CIN_num, const char* birth_date, const char* expiry_date) {
    uint8_t data_buffer[MAX_FRAME_SIZE];
    size_t data_length;

    // 1. Calcul de la clé MRZ
    calculate_mrz_key(CIN_num, birth_date, expiry_date);

    // 2. Authentification BAC
    if (!perform_bac(&pn532)) {
        printf("Échec de l'authentification BAC\n");
        return 0;
    }

    // 3. Lecture des données
    if (!CIN_data(&pn532, data_buffer, &data_length)) {
        printf("Échec de la lecture des données\n");
        return 0;
    }

    // 4. Traitement des données lues
    printf("Données lues avec succès (%d octets)\n", data_length);
    for (size_t i = 0; i < data_length; i++) {
        printf("%02X ", data_buffer[i]);
    }
    printf("\n");

    return 1;
}