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
#include "stm32f4xx_hal_conf.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

//extern I2C_HandleTypeDef hi2c1;
#define PN532_I2C_ADDRESS               0x48
#define HAL_MAX_DELAY                   10

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
    uint8_t params[] = {0x01, 0x14, 0x01};
    //uint8_t params[] = {0x4A, 0x01, 0x00};
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


#define MAX_FRAME_SIZE 256
#define KEY_SIZE 16
#define NONCE_SIZE 8
#define MRZ_SIZE 24

// Structure pour stocker les clés BAC
typedef struct {
    uint8_t Ksenc[16];  // Clé de session pour le chiffrement
    uint8_t Ksmac[16];  // Clé de session pour le MAC
    uint8_t Kenc[16];
    uint8_t Kmac[16];
} BACKeys;

static uint8_t mrz_key[KEY_SIZE];
static BACKeys session_keys;

// Fonctions utilitaires pour le protocole BAC
static void calculate_sha1(const uint8_t* data, size_t length, uint8_t* hash) {
    // Implémentation de SHA-1
    mbedtls_sha1_context ctx;
    mbedtls_sha1_init(&ctx);
    mbedtls_sha1_starts(&ctx);
    mbedtls_sha1_update(&ctx, data, length);
    mbedtls_sha1_finish(&ctx, hash);
    mbedtls_sha1_free(&ctx);
}

static void calculate_3des(const unsigned char *key, const unsigned char *input, size_t input_len, unsigned char *output) {

    mbedtls_des3_context ctx;
	unsigned char iv[8] = {0}; // Initialization vector of zeros

	// Initialize the 3DES context
	mbedtls_des3_init(&ctx);

	// Set the encryption key
	mbedtls_des3_set2key_enc(&ctx, key);

	// Perform 3DES encryption in CBC mode
	int ret = mbedtls_des3_crypt_cbc(&ctx, MBEDTLS_DES_ENCRYPT, input_len, iv, input, output);

	// Free the context
	mbedtls_des3_free(&ctx);
}

// Fonction pour calculer la clé MRZ
static void calculate_mrz_key(const char* CIN_num, const char* birth_date, const char* expiry_date) {
    uint8_t mrz_info[MRZ_SIZE];
    uint8_t hash[20];

    memcpy(mrz_info, CIN_num, 10);
	memcpy(mrz_info + 10, birth_date, 7);
	memcpy(mrz_info + 17, expiry_date, 7);

    // Calculer le SHA-1
    calculate_sha1(mrz_info, MRZ_SIZE, hash);

    // Prendre les 16 premiers octets pour la clé
    memcpy(mrz_key, hash, KEY_SIZE);
    calculate_Kenc_Kmac();
}

static int calculate_KIC(uint8_t* KIC){
	mbedtls_entropy_context entropy;
	mbedtls_ctr_drbg_context ctr_drbg;
	const char *pers = "random_kic_generation";

	// Initialisation des contextes
	mbedtls_entropy_init(&entropy);
	mbedtls_ctr_drbg_init(&ctr_drbg);

	// Initialiser le générateur de nombres aléatoires avec une source d'entropie
	if (mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy, (const unsigned char *) pers, strlen(pers)) != 0) {
		printf("Erreur d'initialisation du generateur de nombres aleatoires\n");
		return 0;
	}

	// Générer les 16 octets aléatoires pour KIC
	if (mbedtls_ctr_drbg_random(&ctr_drbg, KIC, sizeof(KIC)) != 0) {
		printf("Erreur de generation de KIC\n");
		return 0;
	}

	// Libération des contextes
	mbedtls_ctr_drbg_free(&ctr_drbg);
	mbedtls_entropy_free(&entropy);
	return 1;
}
void adjustParity(uint8_t* key, size_t length) {
    for(size_t i = 0; i < length; i++) {
        uint8_t byte = key[i] & 0xFE;  // Clear the parity bit (LSB)
        uint8_t count = 0;

        // Count number of 1s in bits 1-7
        for(int j = 1; j < 8; j++) {
            if(byte & (1 << j)) {
                count++;
            }
        }

        // Set parity bit (LSB) to make total number of 1s odd
        key[i] = byte | (count % 2 == 0 ? 1 : 0);
    }
}

void calculate_Kenc_Kmac(void){
	uint8_t hash1[20];
	uint8_t Enc_D[20] = {0};
	uint8_t Mac_D[20] = {0};
	uint8_t Ka[8];
	uint8_t Kb[8];
	//kenc
	memcpy(Enc_D, mrz_key, 16);
	Enc_D[16] = 0x00;
	Enc_D[17] = 0x00;
	Enc_D[18] = 0x00;
	Enc_D[19] = 0x01;
	calculate_sha1(Enc_D, 20, hash1);
	memcpy(Ka, hash1, 8);
	adjustParity(Ka, 8);
	memcpy(session_keys.Kenc, Ka, 8);
	memcpy(Kb, hash1 + 8, 8);
	adjustParity(Kb, 8);
	memcpy(session_keys.Kenc + 8, Kb, 8);

	//kmac
	memcpy(Mac_D, mrz_key, 16);
	Mac_D[16] = 0x00;
	Mac_D[17] = 0x00;
	Mac_D[18] = 0x00;
	Mac_D[19] = 0x02;
	calculate_sha1(Mac_D, 20, hash1);
	memcpy(Ka, hash1, 8);
	adjustParity(Ka, 8);
	memcpy(session_keys.Kmac, Ka, 8);
	memcpy(Kb, hash1 + 8, 8);
	adjustParity(Kb, 8);
	memcpy(session_keys.Kmac + 8, Kb, 8);

}

// Fonction pour dériver les clés de session
static void derive_session_keys(uint8_t* k_seed) {
    uint8_t hash[20];
    uint8_t enc_data[20] = {0};
    uint8_t mac_data[20] = {0};

    // Dérivation pour Ksenc
	memcpy(enc_data, k_seed, 16);
	enc_data[16] = 0x00;
	enc_data[17] = 0x00;
	enc_data[18] = 0x00;
	enc_data[19] = 0x01;
	calculate_sha1(enc_data, 20, hash);
	memcpy(session_keys.Ksenc, hash, 16);

	// Dérivation pour Ksmac
	memcpy(mac_data, k_seed, 16);
	mac_data[16] = 0x00;
	mac_data[17] = 0x00;
	mac_data[18] = 0x00;
	mac_data[19] = 0x02;
	calculate_sha1(mac_data, 20, hash);
	memcpy(session_keys.Ksmac, hash, 16);

}

void calculate_SSC(uint8_t rnd_icc[8], uint8_t SSC[8]){
	// Copier RND.IC dans SSC
	for (int i = 0; i < 8; i++) {
		SSC[i] = rnd_icc[i];
	}

	// Incrémenter SSC de 1
	for (int i = 7; i >= 0; i--) {
		if (++SSC[i] != 0) break;  // arrêt si pas de dépassement
	}
}


int perform_bac(PN532* pn532) {
    //uint8_t rnd_icc[8], rnd_ifd[8], k_ifd[16];
    uint8_t EIFD[32];
    uint8_t cmd_data[40];
    uint8_t resp_data[40];
    uint8_t response[128];
    uint8_t S[32];
	uint8_t MIFD[8];
	uint8_t KIC[16];

    /*for(int i = 0; i < 8; i++) {
        rnd_ifd[i] = rand() % 256;
    }


    uint8_t get_challenge[] = {0x00, 0x84, 0x00, 0x00, 0x08}; //0x84 | PN532_COMMAND_INDATAEXCHANGE
    if (PN532_CallFunction(pn532, PN532_COMMAND_INDATAEXCHANGE, response, sizeof(response), get_challenge, sizeof(get_challenge), PN532_DEFAULT_TIMEOUT) <= 0) {
        return 0;
    }
    memcpy(rnd_icc, response, 8);*/

    uint8_t rnd_ifd[8] = {0x78, 0x17, 0x23, 0x86, 0x0C, 0x06, 0xC2, 0x26};
    uint8_t rnd_icc[8] = {0x46, 0x08, 0xF9, 0x19, 0x88, 0x70, 0x22, 0x12};
    uint8_t k_ifd[16] = {0x0B, 0x79, 0x52, 0x40, 0xCB, 0x70, 0x49, 0xB0, 0x1C, 0x19, 0xB3, 0x3E, 0x32, 0x80, 0x4F, 0x0B};
    memcpy(S, rnd_ifd, 8);
    memcpy(S + 8, rnd_icc, 8);
    memcpy(S + 16, k_ifd, 16);

    calculate_3des(session_keys.Kenc, S, 32, EIFD);
    compute_retail_mac(session_keys.Kmac, EIFD, 32, MIFD);

    memcpy(cmd_data, EIFD, 32);
    memcpy(cmd_data + 32, MIFD, 8);

    // Envoi de la commande d'authentification
    uint8_t auth_cmd[46] = {0x00, 0x82, 0x00, 0x00, 0x28};
    memcpy(auth_cmd + 5, cmd_data, 40);
    auth_cmd[45] = 0x28;
    if (PN532_CallFunction(pn532, PN532_COMMAND_INDATAEXCHANGE, response, sizeof(response), auth_cmd, sizeof(auth_cmd), PN532_DEFAULT_TIMEOUT) <= 0) {
        return 0;
    }

    if(calculate_KIC(KIC) == 1){
    	printf(" KIC is genereted \n");
    }

    uint8_t k_seed[16];
    for (int i = 0; i < 16; i++) {
    	k_seed[i] = k_ifd[i] ^ KIC[i];
    }

    derive_session_keys(k_seed);

    uint8_t SSC[8];
    calculate_SSC(rnd_icc, SSC);

    uint8_t R[32];
    memcpy(R, rnd_icc, 8);
    memcpy(R + 8, rnd_ifd, 8);
    memcpy(R + 16, KIC, 16);

    uint8_t EIC[32];
    calculate_3des(R, EIC, session_keys.Kenc, MBEDTLS_DES_ENCRYPT);

    uint8_t MIC[8];
    //calculate_cbc_mac(EIC, sizeof(EIC), session_keys.Kmac, MIC);

    memcpy(resp_data, EIC, 32);
    memcpy(resp_data + 32, MIC, 8);
    memcpy(auth_cmd + 5, resp_data, 40);
	auth_cmd[45] = 0x28;
	if (PN532_CallFunction(pn532, PN532_COMMAND_INDATAEXCHANGE, response, sizeof(response), auth_cmd, sizeof(auth_cmd), PN532_DEFAULT_TIMEOUT) <= 0) {
		return 0;
	}

    return 1;
}


int CIN_data(PN532* pn532, uint8_t* data_buffer, size_t* data_length) {
    uint8_t cmd[5] = {0x00, 0xB0, 0x00, 0x00, 0x00};  // READ BINARY
    uint8_t response[128];
    int resp_len;

    uint8_t protected_cmd[MAX_FRAME_SIZE];
    size_t protected_len;

    if (!apply_secure_messaging(cmd, sizeof(cmd), protected_cmd, &protected_len)) {
        return 0;
    }

    resp_len = PN532_CallFunction(pn532, PN532_COMMAND_INDATAEXCHANGE, response, sizeof(response), protected_cmd, protected_len, PN532_DEFAULT_TIMEOUT);
    if (resp_len <= 0) {
        return 0;
    }
    // resp_len / 128
    if (!decrypt_secure_messaging(response, 128, data_buffer, data_length)) {
        return 0;
    }

    return 1;
}


int read_identity_card(PN532* pn532, const char* CIN_num, const char* birth_date, const char* expiry_date) {
    uint8_t data_buffer[MAX_FRAME_SIZE];
    size_t data_length;

    calculate_mrz_key(CIN_num, birth_date, expiry_date);

    if (!perform_bac(pn532)) {
        printf("Echec de l'authentification BAC\n");
        return 0;
    }

    if (!CIN_data(pn532, data_buffer, &data_length)) {
        printf("Echec de la lecture des donnees\n");
        return 0;
    }

    printf("Donnees lues avec succes (%d octets)\n", data_length);
    for (size_t i = 0; i < data_length; i++) {
        printf("%02X ", data_buffer[i]);
    }
    printf("\n");
    printf("Donnes character \n");
	for (size_t i = 0; i < data_length; i++) {
		printf("%c ", data_buffer[i]);
	}

    return 1;
}

// Fonction pour appliquer la messagerie sécurisée
int apply_secure_messaging(uint8_t* cmd, size_t cmd_len, uint8_t* protected_cmd, size_t* protected_len) {
    uint8_t mac[16];
    uint8_t padded_cmd[MAX_FRAME_SIZE];
    size_t padded_len = ((cmd_len + 7) / 8) * 8; // Padding to multiple of 8 bytes

    memcpy(padded_cmd, cmd, cmd_len);
    memset(padded_cmd + cmd_len, 0x00, padded_len - cmd_len);

    mbedtls_cipher_context_t ctx;
    mbedtls_cipher_init(&ctx);
    mbedtls_cipher_setup(&ctx, mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_CBC)); //  MBEDTLS_CIPHER_AES_128_CMAC
    mbedtls_cipher_cmac_starts(&ctx, session_keys.Ksmac, 128);
    mbedtls_cipher_cmac_update(&ctx, padded_cmd, padded_len);
    mbedtls_cipher_cmac_finish(&ctx, mac);
    mbedtls_cipher_free(&ctx);

    memcpy(protected_cmd, cmd, cmd_len);
    memcpy(protected_cmd + cmd_len, mac, 16);
    *protected_len = cmd_len + 16;
    return 1;
}


int decrypt_secure_messaging(uint8_t* encrypted_data, size_t encrypted_len, uint8_t* decrypted_data, size_t* decrypted_len) {
    uint8_t iv[16] = {0}; // Initialisation du vecteur
    mbedtls_aes_context aes_ctx;

    mbedtls_aes_init(&aes_ctx);
    mbedtls_aes_setkey_dec(&aes_ctx, session_keys.Ksenc, 128);

    if (mbedtls_aes_crypt_cbc(&aes_ctx, MBEDTLS_AES_DECRYPT, encrypted_len, iv, encrypted_data, decrypted_data) != 0) {
    	mbedtls_aes_free(&aes_ctx);
        return 0;
    }

    mbedtls_aes_free(&aes_ctx);
    *decrypted_len = encrypted_len;

    return 1;
}

void compute_retail_mac(const unsigned char *key, const unsigned char *input, size_t input_len, unsigned char *mac) {
    mbedtls_des_context des_ctx;
    mbedtls_des3_context des3_ctx;
    unsigned char iv[8] = {0};    // Initial IV of zeros
    unsigned char tmp[8] = {0};   // Temporary buffer for calculations

    // Initialize contexts
    mbedtls_des_init(&des_ctx);
    mbedtls_des3_init(&des3_ctx);

    // Split the 16-byte key into K1 (first 8 bytes) and K2 (last 8 bytes)
    const unsigned char *K1 = key;
    const unsigned char *K2 = key + 8;

    // Process all blocks except the last with single DES using K1
    mbedtls_des_setkey_enc(&des_ctx, K1);

    for(size_t i = 0; i < input_len/8 - 1; i++) {
        // XOR with previous result (or IV for first block)
        for(int j = 0; j < 8; j++) {
            tmp[j] = iv[j] ^ input[i * 8 + j];
        }

        // DES encrypt with K1
        mbedtls_des_crypt_ecb(&des_ctx, tmp, iv);
    }

    // Process last block specially (Retail-MAC specific)
    // XOR last input block with previous result
    for(int j = 0; j < 8; j++) {
        tmp[j] = iv[j] ^ input[(input_len/8 - 1) * 8 + j];
    }

    // For last block: DES decrypt with K2, then DES encrypt with K1
    mbedtls_des_setkey_dec(&des_ctx, K2);
    mbedtls_des_crypt_ecb(&des_ctx, tmp, tmp);

    mbedtls_des_setkey_enc(&des_ctx, K1);
    mbedtls_des_crypt_ecb(&des_ctx, tmp, mac);

    // Clean up
    mbedtls_des_free(&des_ctx);
    mbedtls_des3_free(&des3_ctx);

}
