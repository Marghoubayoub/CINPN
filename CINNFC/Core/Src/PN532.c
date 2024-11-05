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

// Structure pour stocker les cles BAC
typedef struct {
    uint8_t Ksenc[16];  // Clé de session pour le chiffrement
    uint8_t Ksmac[16];  // Clé de session pour le MAC
    uint8_t Kenc[16];
    uint8_t Kmac[16];
} BACKeys;

static uint8_t mrz_key[KEY_SIZE];
static BACKeys session_keys;
uint8_t SSC[8];

// Fonctions utilitaires pour le protocole BAC
static void calculate_sha1(const uint8_t* data, size_t length, uint8_t* hash) {
    // Implementation de SHA-1
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

	mbedtls_des3_set2key_enc(&ctx, key);

	// Perform 3DES encryption in CBC mode
	mbedtls_des3_crypt_cbc(&ctx, MBEDTLS_DES_ENCRYPT, input_len, iv, input, output);

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
    uint8_t Ka[8];
    uint8_t Kb[8];

    // Dérivation pour Ksenc
	memcpy(enc_data, k_seed, 16);
	enc_data[16] = 0x00;
	enc_data[17] = 0x00;
	enc_data[18] = 0x00;
	enc_data[19] = 0x01;
	calculate_sha1(enc_data, 20, hash);
	memcpy(Ka, hash, 8);
	adjustParity(Ka, 8);
	memcpy(session_keys.Ksenc, Ka, 8);
	memcpy(Kb, hash + 8, 8);
	adjustParity(Kb, 8);
	memcpy(session_keys.Ksenc + 8, Kb, 8);

	// Dérivation pour Ksmac
	memcpy(mac_data, k_seed, 16);
	mac_data[16] = 0x00;
	mac_data[17] = 0x00;
	mac_data[18] = 0x00;
	mac_data[19] = 0x02;
	calculate_sha1(mac_data, 20, hash);
	memcpy(Ka, hash, 8);
	adjustParity(Ka, 8);
	memcpy(session_keys.Ksmac, Ka, 8);
	memcpy(Kb, hash + 8, 8);
	adjustParity(Kb, 8);
	memcpy(session_keys.Ksmac + 8, Kb, 8);

}

void calculate_SSC(
    const unsigned char *rnd_ic,
    const unsigned char *rnd_ifd,
    unsigned char *ssc)
{
    // Take last 4 bytes of rnd_ic
    memcpy(ssc, rnd_ic + 4, 4);

    // Take last 4 bytes of rnd_ifd
    memcpy(ssc + 4, rnd_ifd + 4, 4);
}


int make_bac(PN532* pn532) {
    //uint8_t rnd_icc[8], rnd_ifd[8], k_ifd[16];
    uint8_t EIFD[32];
    uint8_t cmd_data[40];
    uint8_t resp_data[40];
    uint8_t response[128];
    uint8_t S[32];
	uint8_t MIFD[8];
	//uint8_t KIC[16];

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
    calculate_mac(EIFD, 32, session_keys.Kmac, 16);

    memcpy(cmd_data, EIFD, 32);
    memcpy(cmd_data + 32, MIFD, 8);

    // Envoi de la commande d'authentification
    uint8_t auth_cmd[46] = {0x00, 0x82, 0x00, 0x00, 0x28};
    memcpy(auth_cmd + 5, cmd_data, 40);
    auth_cmd[45] = 0x28;
    if (PN532_CallFunction(pn532, PN532_COMMAND_INDATAEXCHANGE, response, sizeof(response), auth_cmd, sizeof(auth_cmd), PN532_DEFAULT_TIMEOUT) <= 0) {
        return 0;
    }

    /*get_challenge[] = {0x00, 0x84, 0x00, 0x00, 0x10}; //0x84 | PN532_COMMAND_INDATAEXCHANGE
	if (PN532_CallFunction(pn532, PN532_COMMAND_INDATAEXCHANGE, response, sizeof(response), get_challenge, sizeof(get_challenge), PN532_DEFAULT_TIMEOUT) <= 0) {
		return 0;
	}
	memcpy(KIC, response, 16);*/
    uint8_t KIC[16] = {0x0B, 0x4F, 0x80, 0x32, 0x3E, 0xB3, 0x19, 0x1C, 0xB0, 0x49, 0x70, 0xCB, 0x40, 0x52, 0x79, 0x0B};
    uint8_t k_seed[16];
    for (int i = 0; i < 16; i++) {
    	k_seed[i] = k_ifd[i] ^ KIC[i];
    }

    derive_session_keys(k_seed);

    calculate_SSC(rnd_icc, rnd_ifd, SSC);

    uint8_t R[32];
    memcpy(R, rnd_icc, 8);
    memcpy(R + 8, rnd_ifd, 8);
    memcpy(R + 16, KIC, 16);

    uint8_t EIC[32];
    calculate_3des(session_keys.Kenc, R, 32, EIC);

    uint8_t MIC[8];
    compute_mac_over_eifd(EIC, sizeof(EIC), session_keys.Kmac, MIC, sizeof(MIC));

    memcpy(resp_data, EIC, 32);
    memcpy(resp_data + 32, MIC, 8);
    memcpy(auth_cmd + 5, resp_data, 40);
	auth_cmd[45] = 0x28;
	/*if (PN532_CallFunction(pn532, PN532_COMMAND_INDATAEXCHANGE, response, sizeof(response), auth_cmd, sizeof(auth_cmd), PN532_DEFAULT_TIMEOUT) <= 0) {
		return 0;
	}*/

    return 1;
}

void incm_SSC(uint8_t* SSC, uint8_t len_SSC){
    for(uint8_t i = len_SSC - 1; i >= 0; i--){
        if( SSC[i] == 0xFF){
            SSC[i] = 0x00;
        }
        else{
            SSC[i]++;
            break;
        }
    }
}

decrypt_data(uint8_t* data, size_t len_data, uint8_t* Ksenc, uint8_t* data_decrypted, size_t len_data_decrypted){
	mbedtls_des3_context ctx;
	unsigned char iv[8] = {0};
	mbedtls_des3_init(&ctx);
	mbedtls_des3_set2key_dec(&ctx, Ksenc);
	mbedtls_des3_crypt_cbc(&ctx, MBEDTLS_DES_DECRYPT, len_data, iv, data, data_decrypted);
}

int CIN_data(PN532* pn532, uint8_t* data_buffer, size_t* data_length) {
	// I- Selectionner EF.COM
    uint8_t Cmd_Header[8] = {0x0C, 0xA4, 0x02, 0x0C, 0x80, 0x00, 0x00, 0x00};
    uint8_t Data[8] = {0x01, 0x1E, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t remp[5] = {0x80, 0x00, 0x00, 0x00, 0x00};

    uint8_t Data_ecrypted[8];
    calculate_3des(session_keys.Ksenc, Data, 8, Data_ecrypted);

    uint8_t DO87[11];
    DO87[0] = 0x87;
    DO87[1] = 0x09;
    DO87[2] = 0x01;
    memcpy(DO87 + 3, Data_ecrypted, 8);

    uint8_t M[19];
    memcpy(M, Cmd_Header, 8);
    memcpy(M + 8, DO87, 11);

    incm_SSC(SSC, sizeof(SSC));
    uint8_t N[32];
    memcpy(N, SSC, 8);
    memcpy(N + 8, M, 19);
    memcpy(N + 27, remp, 5);


    // CC MAC of N with ksmac
    uint8_t CC[8] = {0xBF, 0x8B, 0x92, 0xD6, 0x35, 0xFF, 0x24, 0xF8};
    //mbedtls_cipher_cmac( mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB), session_keys.Ksmac, 128, N, sizeof(N), CC);

    uint8_t DO8E[10];
    DO8E[0] = 0x8E;
    DO8E[1] = 0x08;
    memcpy(DO8E + 2, CC, 8);

    uint8_t ProtectedAPDU[27];
    memcpy(ProtectedAPDU, Cmd_Header, 5);
    memcpy(ProtectedAPDU + 5, DO87, 11);
    memcpy(ProtectedAPDU + 16, DO8E, 10);
    ProtectedAPDU[26] = 0x00;
    uint8_t RAPDU[16];
    /*if (PN532_CallFunction(pn532, PN532_COMMAND_INDATAEXCHANGE, RAPDU, sizeof(RAPDU), ProtectedAPDU, sizeof(ProtectedAPDU), PN532_DEFAULT_TIMEOUT) <= 0) {
    		return 0;
    }*/

    incm_SSC(SSC, sizeof(SSC));
    uint8_t DO99[8] = {0x99, 0x02, 0x90, 0x00, 0x80, 0x00, 0x00, 0x00};
    uint8_t K[16];
    memcpy(K, SSC, 8);
    memcpy(K + 8, DO99, 8);

    // CC1 MAC of K with ksmac
    uint8_t CC1[8] = {0xFA, 0x85, 0x5A, 0x5D, 0x4C, 0x50, 0xA8, 0xED};
    //Comparer CC1 avec les données de DO’8E’ de RAPDU


    // II- Lire les elements binaires des quatre premiers octets :
    uint8_t CmdHeader1[8] = {0x0C, 0xB0, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00};
    uint8_t DO97[3] = {0x97, 0x01, 0x04};
    uint8_t M1[11];
    memcpy(M1, CmdHeader1, 8);
    memcpy(M1 + 8, DO97, 3);

    incm_SSC(SSC, sizeof(SSC));
    uint8_t N1[24];
    memcpy(N1, SSC, 8);
    memcpy(N1 + 8, M1, 11);
    memcpy(N1 + 19, remp, 5);
    //Calculer le MAC CC2 sur N1 avec KSMAC
    uint8_t CC2[8] = {0xED, 0x67, 0x05, 0x41, 0x7E, 0x96, 0xBA, 0x55};

    uint8_t DO8E1[10];
    DO8E1[0] = 0x8E;
	DO8E1[1] = 0x08;
	memcpy(DO8E1 + 2, CC2, 8);

	uint8_t ProtectedAPDU1[19];
	memcpy(ProtectedAPDU1, CmdHeader1, 4);
	ProtectedAPDU1[4] = 0x0D;
	memcpy(ProtectedAPDU1 + 5, DO97, 3);
	memcpy(ProtectedAPDU1 + 8, DO8E1, 10);
	ProtectedAPDU1[18] = 0x00;
	uint8_t RAPDU1[27] = {0x87, 0x09, 0x01, 0x9F, 0xF0, 0xEC, 0x34, 0xF9, 0x92, 0x26, 0x51, 0x99, 0x02, 0x90, 0x00,
			 0x8E, 0x08, 0xAD, 0x55, 0xCC, 0x17, 0x14, 0x0B, 0x2D, 0xED, 0x90, 0x00};
	/*if (PN532_CallFunction(pn532, PN532_COMMAND_INDATAEXCHANGE, RAPDU1, sizeof(RAPDU1), ProtectedAPDU1, sizeof(ProtectedAPDU1), PN532_DEFAULT_TIMEOUT) <= 0) {
	    		return 0;
	}*/

	incm_SSC(SSC, sizeof(SSC));
	uint8_t K1[24];
	memcpy(K1, SSC, 8);
	memcpy(K1 + 8, RAPDU1, 11);
	memcpy(K1 + 19, DO99, 5);

	//Calculer MAC CC3 avec KSMAC
	uint8_t DO87_data[8];
	memcpy(DO87_data, RAPDU1 + 3, 8);
	uint8_t data_decrypted[4];
	decrypt_data(DO87_data, sizeof(DO87_data), session_keys.Ksenc, data_decrypted, sizeof(data_decrypted));


	// III-  Lire les elements binaires des 18 octets restants a partir du décalage 4
	uint8_t CmdHeader2[8] = {0x0C, 0xB0, 0x00, 0x04, 0x80, 0x00, 0x00, 0x00};
	uint8_t DO971[3] = {0x97, 0x01, 0x12};
	uint8_t M2[11];
	memcpy(M2, CmdHeader2, 8);
	memcpy(M2 + 8, DO971, 3);

	incm_SSC(SSC, sizeof(SSC));
	uint8_t N2[24];
	memcpy(N2, SSC, 8);
	memcpy(N2 + 8, M2, 11);
	memcpy(N2 + 19, remp, 5);

	//Calculer le MAC CC4 sur N avec KSMAC
	uint8_t CC4[8] = {0x2E, 0xA2, 0x8A, 0x70, 0xF3, 0xC7, 0xB5, 0x35};
	uint8_t DO8E2[10];
	DO8E2[0] = 0x8E;
	DO8E2[1] = 0x08;
	memcpy(DO8E2 + 2, CC4, 8);

	uint8_t ProtectedAPDU2[19];
	memcpy(ProtectedAPDU2, CmdHeader2, 4);
	ProtectedAPDU2[4] = 0x0D;
	memcpy(ProtectedAPDU2 + 5, DO971, 3);
	memcpy(ProtectedAPDU2 + 8, DO8E2, 10);
	ProtectedAPDU2[18] = 0x00;
	uint8_t RAPDU2[43] = {0x87, 0x19, 0x01, 0xFB, 0x92, 0x35, 0xF4, 0xE4, 0x03, 0x7F, 0x23, 0x27, 0xDC, 0xC8, 0x96, 0x4F,
			0x1F, 0x9B, 0x8C, 0x30, 0xF4, 0x2C, 0x8E, 0x2F, 0xFF, 0x22, 0x4A, 0x99, 0x02, 0x90, 0x00, 0x8E, 0x08, 0xC8, 0xB2, 0x78, 0x7E, 0xAE, 0xA0, 0x7D, 0x74, 0x90, 0x00};
	/*if (PN532_CallFunction(pn532, PN532_COMMAND_INDATAEXCHANGE, RAPDU2, sizeof(RAPDU2), ProtectedAPDU2, sizeof(ProtectedAPDU2), PN532_DEFAULT_TIMEOUT) <= 0) {
		  return 0;
	}*/

	incm_SSC(SSC, sizeof(SSC));
	uint8_t K2[39];
	memcpy(K2, SSC, 8);
	memcpy(K2 + 8, RAPDU2, 27);
	memcpy(K2 + 35, DO99, 4);

	//Calculer le MAC CC5 avec KSMAC
	uint8_t CC5[8] = {0xC8, 0xB2, 0x78, 0x7E, 0xAE, 0xA0, 0x7D, 0x74};
	uint8_t DO87_data1[24];
	memcpy(DO87_data1, RAPDU2 + 3, 24);
	uint8_t data_decrypted1[18];
	decrypt_data(DO87_data1, sizeof(DO87_data1), session_keys.Ksenc, data_decrypted1, sizeof(data_decrypted1));



    return 1;
}


int read_identity_card(PN532* pn532, const char* CIN_num, const char* birth_date, const char* expiry_date) {
    uint8_t data_buffer[MAX_FRAME_SIZE];
    size_t data_length;

    calculate_mrz_key(CIN_num, birth_date, expiry_date);

    if (!make_bac(pn532)) {
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

//void configure_des(const unsigned char *key, const unsigned char *input, unsigned char *output) {
//
//}

void des_ecb_crypt(unsigned char* input, unsigned char* output, unsigned char* key) {
	mbedtls_cipher_context_t ctx;
	mbedtls_cipher_init(&ctx);

//	mbedtls_cipher_setup(&ctx, mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_DES_CBC));

//	mbedtls_cipher_cmac(mbedtls_cipher_info_from_values( MBEDTLS_CIPHER_ID_DES, 64, MBEDTLS_MODE_CBC), key, 8, input, 8, output);
	mbedtls_cipher_cmac(mbedtls_cipher_info_from_values( MBEDTLS_CIPHER_ID_AES, 128, MBEDTLS_MODE_CBC), key, 8, input, 8, output);

//	mbedtls_cipher_cmac_starts(&ctx, key, 128);
//	mbedtls_cipher_cmac_update(&ctx, input, 8);
//	mbedtls_cipher_cmac_finish(&ctx, output);


}

void xor_block(unsigned char* src, unsigned char* dest) {

    for (int x = 0; x < 8; x++) {
       src[x] =  src[x] ^ dest[x];
    }
}

void calculate_mac(uint8_t* eifd, size_t eifd_len, uint8_t* kmac, size_t *mac_len){

//	uint8_t CC[8];
//	uint8_t paddedEifd[32+8];
//	memcpy(paddedEifd, eifd, 32);
//	paddedEifd[32] = 0x80;
//	memset(paddedEifd + 33, 0, 7);
//
//	mbedtls_cipher_context_t ctx;
//	mbedtls_cipher_init(&ctx);
//	mbedtls_cipher_setup(&ctx, mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB));
//	mbedtls_cipher_cmac_starts(&ctx, kmac, 128);
//	mbedtls_cipher_cmac_update(&ctx, paddedEifd, eifd_len);
//	mbedtls_cipher_cmac_finish(&ctx, CC);
//
//	CC;



	const unsigned char msg[40] = { 0x72, 0xC2, 0x9C, 0x23, 0x71, 0xCC, 0x9B, 0xDB,
	                              0x65, 0xB7, 0x79, 0xB8, 0xE8, 0xD3, 0x7B, 0x29,
	                              0xEC, 0xC1, 0x54, 0xAA, 0x56, 0xA8, 0x79, 0x9F,
	                              0xAE, 0x2F, 0x49, 0x8F, 0x76, 0xED, 0x92, 0xF2,
	                              0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	//initialization vector
	unsigned char iv[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	unsigned char k0[16] = { 0x79, 0x62, 0xD9, 0xEC, 0xE0, 0x3D, 0x1A, 0xCD, 0x4C, 0x76, 0x08, 0x9D, 0xCE, 0x13, 0x15, 0x43};

	uint8_t myout[8] = {0};

	mbedtls_cipher_context_t ctx;
	mbedtls_cipher_init(&ctx);
	int ret = 0;

#define MBEDTLS_CMAC_C

	const mbedtls_cipher_info_t *cipher_info;
	cipher_info = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB);

	if ((ret = mbedtls_cipher_setup(&ctx, cipher_info)) != 0) {
		ret;
	}

	ret = mbedtls_cipher_cmac_starts(&ctx, k0, 128);
	if (ret != 0) {
		ret;
	}

	ret = mbedtls_cipher_cmac_update(&ctx, msg, 40);
	if (ret != 0) {
		ret;
	}

	ret = mbedtls_cipher_cmac_finish(&ctx, myout);
	if(ret != 0) {
		ret;
	}

//	int res = mbedtls_cipher_cmac(mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB), k0, 128, msg, 40, myout);
//	if(res) {
//		while(1){
//			res;
//		}
//	}

//	unsigned char output[8];
//	unsigned char xx[8];
//	unsigned char block[8];
//	int offset = 0;
//
//	memcpy(xx, iv, 8);
//
//	// Chain and encrypt 5 8-bit blocks
//	for (int x = 0; x < 5; x++) {
//
//		memcpy(block, &msg[offset] , 8);
//		offset+=8;
//
//		//set xx `xor {xx} {mj}` # chain
//		xor_block(xx, block);
//
//		//set xx `des -k {k0} -c {xx}` #encrypt
//		des_ecb_crypt(xx, output, k0);
//		memcpy(xx, output, 8);
//	}
//
//
//	des_ecb_crypt(xx, output, k1);
//	memcpy(xx, output, 8);
//
//	des_ecb_crypt(xx, output, k0);
//	memcpy(xx, output, 8);



//	uint8_t kmac1[8];
//	uint8_t kmac2[8];
//
//	memcpy(kmac1, kmac, 8);
//	memcpy(kmac2, kmac + 8, 8);
//
//	uint8_t paddedeifd[40] = {0};
//	memcpy(paddedeifd, eifd, eifd_len);
//	paddedeifd[32] = 0x80;
////	memset(paddedeifd + 33, 0, 7);
//	size_t padded_len = eifd_len + 1;
//	while (padded_len % 8 != 0) {
//		paddedeifd[padded_len++] = 0x00;  // Compléter avec des zéros
//	}
//
//	uint8_t d1[8];
//	uint8_t d2[8];
//	uint8_t d3[8];
//	uint8_t d4[8];
//	uint8_t d5[8];
//
//	memcpy(d1, paddedeifd, 8);
//	memcpy(d2, paddedeifd + 8, 8);
//	memcpy(d3, paddedeifd + 16, 8);
//	memcpy(d4, paddedeifd + 24, 8);
//	memcpy(d5, paddedeifd + 32, 8);
//
//	mbedtls_des3_context ctx;
//
//	mbedtls_des3_crypt_ecb(&ctx, input, output);
//
//	unsigned char iv[8] = {0};
//
//	mbedtls_des_setkey_enc(&des1, kmac1);
//	mbedtls_des_setkey_enc(&des2, kmac2);
//
//
//	uint8_t h1[8] = {0};
//	uint8_t h2[8] = {0};
//	uint8_t h3[8] = {0};
//	uint8_t h4[8] = {0};
//	uint8_t h5[8] = {0};
//	unsigned char int_block[8];
//
//
//
//	mbedtls_des_crypt_cbc(&des1, MBEDTLS_DES_ENCRYPT, 8, iv, d1, h1);
//	//configure_des(kmac1, d1, h1);
//
//	for(uint8_t i = 0; i < 8; i++){
//		int_block[i] = (uint8_t)(h1[i] ^ d2[i]);
//	}
//	mbedtls_des_crypt_cbc(&des1, MBEDTLS_DES_ENCRYPT, 8, iv, int_block, h2);
//	//configure_des(kmac1, int2, h2);
//	for(uint8_t i = 0; i < 8; i++){
//		int_block[i] = (uint8_t)(h2[i] ^ d3[i]);
//	}
//	mbedtls_des_crypt_cbc(&des1, MBEDTLS_DES_ENCRYPT, 8, iv, int_block, h3);
//	//configure_des(kmac1, int3, h3);
//	for(uint8_t i = 0; i < 8; i++){
//		int_block[i] = (uint8_t)(h3[i] ^ d4[i]);
//	}
//	mbedtls_des_crypt_cbc(&des1, MBEDTLS_DES_ENCRYPT, 8, iv, int_block, h4);
//	//configure_des(kmac1, int4, h4);
//	for(uint8_t i = 0; i < 8; i++){
//		int_block[i] = (uint8_t)(h4[i] ^ d5[i]);
//	}
//	mbedtls_des_crypt_cbc(&des1, MBEDTLS_DES_ENCRYPT, 8, iv, int_block, h5);
//	//configure_des(kmac1, int5, h5);
//
//	uint8_t h5decrypt[8];
//
//	mbedtls_des_crypt_cbc(&des2, MBEDTLS_DES_ENCRYPT, 8, iv, h5, h5decrypt);
//	//configure_des(kmac2, h5, h5decrypt);
//
//	uint8_t MAC[8];
//
//	mbedtls_des_setkey_dec(&des2, kmac2);  // Configurer pour déchiffrement
//	mbedtls_des_crypt_ecb(&des2, h5, h5decrypt);
//
//	mbedtls_des_setkey_enc(&des1, kmac1);  // Revenir au chiffrement
//	mbedtls_des_crypt_ecb(&des1, h5decrypt, MAC);
//
//	//mbedtls_des3_crypt_cbc(&des1, MBEDTLS_DES_ENCRYPT, 8, iv, h5decrypt, MAC);
//	//configure_des(kmac1, h5decrypt, MAC);
//	MAC;


}

void compute_mac_over_eifd(
    const unsigned char *eifd,
    size_t eifd_len,
    const unsigned char *kmac,
    unsigned char *mac_output,
    size_t *mac_len)
{
    int ret = 0;
    mbedtls_des3_context ctx;
    unsigned char iv[8] = {0};
    unsigned char tmp[8] = {0};

    // Initialize context
    mbedtls_des3_init(&ctx);

    // Set the MAC key
    ret = mbedtls_des3_set2key_enc(&ctx, kmac);
    if (ret != 0) {
        goto cleanup;
    }

    // Process all blocks in CBC mode
    size_t remaining = eifd_len;
    const unsigned char *curr_input = eifd;

    while (remaining > 8) {
        // XOR with previous block
        for (int i = 0; i < 8; i++) {
            tmp[i] = iv[i] ^ curr_input[i];
        }

        // Encrypt
        ret = mbedtls_des3_crypt_ecb(&ctx, tmp, iv);
        if (ret != 0) {
            goto cleanup;
        }

        curr_input += 8;
        remaining -= 8;
    }

    // Process last block
    if (remaining > 0) {
        // XOR with previous block
        for (int i = 0; i < 8; i++) {
            tmp[i] = iv[i] ^ curr_input[i];
        }

        // Final encryption
        ret = mbedtls_des3_crypt_ecb(&ctx, tmp, mac_output);
        if (ret != 0) {
            goto cleanup;
        }
    }

    *mac_len = 8;

cleanup:
    mbedtls_des3_free(&ctx);
    mbedtls_platform_zeroize(iv, sizeof(iv));
    mbedtls_platform_zeroize(tmp, sizeof(tmp));

}

/*void compute_mac_over_eifd(
	    const unsigned char *eifd,      // Input: EIFD (encrypted signature)
	    size_t eifd_len,               // Length of EIFD
	    const unsigned char *kmac,      // 16-byte key for MAC (K1||K2)
	    unsigned char *mac_output,      // Output: 8-byte MAC
	    size_t *mac_len)               // Output MAC length
	{
	    int ret = 0;
	    mbedtls_des_context des_ctx;
	    unsigned char iv[8] = {0};      // Initialize IV with zeros
	    unsigned char tmp[8] = {0};     // Temporary buffer for calculations

	    // Initialize DES context
	    mbedtls_des_init(&des_ctx);

	    // Split 16-byte key into K1 (first 8 bytes) and K2 (last 8 bytes)
	    const unsigned char *K1 = kmac;
	    const unsigned char *K2 = kmac + 8;

	    // Process all blocks except the last with single DES using K1
	    ret = mbedtls_des_setkey_enc(&des_ctx, K1);
	    if (ret != 0) {
	        goto cleanup;
	    }

	    // Process all blocks except the last
	    size_t blocks = eifd_len / 8;
	    for(size_t i = 0; i < blocks - 1; i++) {
	        // XOR with previous result
	        for(int j = 0; j < 8; j++) {
	            tmp[j] = iv[j] ^ eifd[i * 8 + j];
	        }

	        // DES encrypt with K1
	        ret = mbedtls_des_crypt_ecb(&des_ctx, tmp, iv);
	        if (ret != 0) {
	            goto cleanup;
	        }
	    }

	    // Process last block specially (Retail-MAC)
	    // First: XOR last block with previous result
	    for(int j = 0; j < 8; j++) {
	        tmp[j] = iv[j] ^ eifd[(blocks - 1) * 8 + j];
	    }

	    // Then: DES decrypt with K2
	    ret = mbedtls_des_setkey_dec(&des_ctx, K2);
	    if (ret != 0) {
	        goto cleanup;
	    }
	    ret = mbedtls_des_crypt_ecb(&des_ctx, tmp, tmp);
	    if (ret != 0) {
	        goto cleanup;
	    }

	    // Finally: DES encrypt with K1
	    ret = mbedtls_des_setkey_enc(&des_ctx, K1);
	    if (ret != 0) {
	        goto cleanup;
	    }
	    ret = mbedtls_des_crypt_ecb(&des_ctx, tmp, mac_output);
	    if (ret != 0) {
	        goto cleanup;
	    }

	    *mac_len = 8;

	cleanup:
	    mbedtls_des_free(&des_ctx);
	    mbedtls_platform_zeroize(tmp, sizeof(tmp));
	    mbedtls_platform_zeroize(iv, sizeof(iv));

	}*/


/*static int calculate_KIC(uint8_t* KIC){
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
}*/
