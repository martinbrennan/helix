/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <string.h>
#include "esp_log.h"
#include "i2c_bus.h"
#include "es8388.h"
//#include "board.h"
//#include "audio_volume.h"

#include <helix.h>

void xinit(void);


// MJB from audio_gpio.h

typedef int i2s_port_t;	
typedef int i2c_port_t;

/// I2S Pins
typedef struct {
    gpio_num_t bck_io_num;
    gpio_num_t ws_io_num;
    gpio_num_t data_out_num;
    gpio_num_t data_in_num;
} i2s_pin_config_t;

// SPI Configuration
typedef struct {
    gpio_num_t mosi_io_num;    ///< GPIO pin for Master Out Slave In (=spi_d) signal, or -1 if not used.
    gpio_num_t miso_io_num;    ///< GPIO pin for Master In Slave Out (=spi_q) signal, or -1 if not used.
    gpio_num_t sclk_io_num;      ///< GPIO pin for SPI Clock signal, or -1 if not used.
    gpio_num_t quadwp_io_num;
    gpio_num_t quadhd_io_num;
} spi_bus_config_t;

#include "board_pins_config.h"


#ifdef CONFIG_ESP_LYRAT_V4_3_BOARD
#include "headphone_detect.h"
#endif

static const char *ES_TAG = "ES8388_DRIVER";
static i2c_bus_handle_t i2c_handle;
// MJB static codec_dac_volume_config_t *dac_vol_handle;

#define ES8388_DAC_VOL_CFG_DEFAULT() {                      \
    .max_dac_volume = 0,                                    \
    .min_dac_volume = -96,                                  \
    .board_pa_gain = BOARD_PA_GAIN,                         \
    .volume_accuracy = 0.5,                                 \
    .dac_vol_symbol = -1,                                   \
    .zero_volume_reg = 0,                                   \
    .reg_value = 0,                                         \
    .user_volume = 0,                                       \
    .offset_conv_volume = NULL,                             \
}

#define ES_ASSERT(a, format, b, ...) \
    if ((a) != 0) { \
        ESP_LOGE(ES_TAG, format, ##__VA_ARGS__); \
        return b;\
    }

audio_hal_func_t AUDIO_CODEC_ES8388_DEFAULT_HANDLE = {
    .audio_codec_initialize = NULL,
    .audio_codec_deinitialize = es8388_deinit,
    .audio_codec_ctrl = es8388_ctrl_state,
    .audio_codec_config_iface = es8388_config_i2s,
    .audio_codec_set_mute = es8388_set_voice_mute,
    .audio_codec_set_volume = es8388_set_voice_volume,
    .audio_codec_get_volume = es8388_get_voice_volume,
//    .audio_hal_lock = NULL,
    .handle = NULL,
};

static esp_err_t es_write_reg(uint8_t slave_addr, uint8_t reg_add, uint8_t data)
{
    return i2c_bus_write_bytes(i2c_handle, slave_addr, &reg_add, sizeof(reg_add), &data, sizeof(data));
}

static esp_err_t es_read_reg(uint8_t reg_add, uint8_t *p_data)
{
    return i2c_bus_read_bytes(i2c_handle, ES8388_ADDR, &reg_add, sizeof(reg_add), p_data, 1);
}



static int i2c_init()
{
//    int res;
    i2c_config_t es_i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
        .sda_io_num = GPIO_NUM_39,		// MJB S3
        .scl_io_num = GPIO_NUM_1		// MJB S3
    };
// MJB    res = get_i2c_pins(I2C_NUM_0, &es_i2c_cfg);
//    ES_ASSERT(res, "getting i2c pins error", -1);
    i2c_handle = i2c_bus_create(I2C_NUM_0, &es_i2c_cfg);
    return 1;
}

/* MJB
void es8388_read_all()
{
    for (int i = 0; i < 50; i++) {
        uint8_t reg = 0;
        es_read_reg(i, &reg);
        ets_printf("%x: %x\n", i, reg);
    }
}
*/

esp_err_t es8388_write_reg(uint8_t reg_add, uint8_t data)
{
    return es_write_reg(ES8388_ADDR, reg_add, data);
}

/**
 * @brief Configure ES8388 ADC and DAC volume. Basicly you can consider this as ADC and DAC gain
 *
 * @param mode:             set ADC or DAC or all
 * @param volume:           -96 ~ 0              for example Es8388SetAdcDacVolume(ES_MODULE_ADC, 30, 6); means set ADC volume -30.5db
 * @param dot:              whether include 0.5. for example Es8388SetAdcDacVolume(ES_MODULE_ADC, 30, 4); means set ADC volume -30db
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
static int es8388_set_adc_dac_volume(int mode, int volume, int dot)
{
    int res = 0;
    if ( volume < -96 || volume > 0 ) {
        ESP_LOGW(ES_TAG, "Warning: volume < -96! or > 0!\n");
        if (volume < -96)
            volume = -96;
        else
            volume = 0;
    }
    dot = (dot >= 5 ? 1 : 0);
    volume = (-volume << 1) + dot;
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL8, volume);
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL9, volume);  //ADC Right Volume=0db
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL5, volume);
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL4, volume);
    }
    return res;
}


// MJB new function

/*
// volume 0-100
esp_err_t es8388_set_adc_gain (int volume)
{
	esp_err_t res = ESP_OK;
    KIT_LOGE("MJB es8388_set_adc_gain: %d", volume);
    KIT_LOGD("MJB es8388_set_adc_gain: %d", volume);

    uint8_t reg = 0;
    es_read_reg(ES8388_ADCCONTROL8, &reg);
    KIT_LOGE("MJB current attenuation: %d", reg);

    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, 0xbb);

    es_read_reg(ES8388_ADCCONTROL1, &reg);
    KIT_LOGE("MJB current gain: %x", reg);

// map input volume 0-100 to attenuation 191 to 0

	int attenuation = -(volume * 2);			// 0 to -200
	attenuation += 200;							// 200 to 0
	if (attenuation > 191) attenuation = 191;	// 191 to 0

    KIT_LOGE("MJB attenuation: %d", attenuation);
    
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL8, attenuation);
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL9, attenuation);  //ADC Right Volume=0db    
    return res;
}    
*/


void regDump (){

	uint8_t regs[16];
//	uint8_t reg_add = 0;

/*	
    KIT_LOGE ("burst");
    i2c_bus_read_bytes(i2c_handle, ES8388_ADDR, &reg_add, sizeof(reg_add), regs, 1);
    KIT_LOGE("%02x %02x %02x %02x %02x %02x %02x %02x", regs[0],regs[1],regs[2],regs[3],regs[4],regs[5],regs[6],regs[7]);    
    KIT_LOGE("%02x %02x %02x %02x %02x %02x %02x %02x", regs[8],regs[9],regs[10],regs[11],regs[12],regs[13],regs[14],regs[15]); 

    KIT_LOGE ("burst+1");
	reg_add = 1;
    i2c_bus_read_bytes(i2c_handle, ES8388_ADDR, &reg_add, sizeof(reg_add), regs, 1);
    KIT_LOGE("%02x %02x %02x %02x %02x %02x %02x %02x", regs[0],regs[1],regs[2],regs[3],regs[4],regs[5],regs[6],regs[7]);    
    KIT_LOGE("%02x %02x %02x %02x %02x %02x %02x %02x", regs[8],regs[9],regs[10],regs[11],regs[12],regs[13],regs[14],regs[15]);
*/    
    int n;
    
    for (n=0;n<16;n++)
		es_read_reg(n, &regs[n]);		
    printf ("Individual\n");
    printf ("%02x %02x %02x %02x %02x %02x %02x %02x\n", regs[0],regs[1],regs[2],regs[3],regs[4],regs[5],regs[6],regs[7]);    
    printf ("%02x %02x %02x %02x %02x %02x %02x %02x\n", regs[8],regs[9],regs[10],regs[11],regs[12],regs[13],regs[14],regs[15]);		
    
    
}	



// gain = 0-10

 
void es8388_set_adc_gain (int gain)
{
    printf("MJB es8388_set_adc_gain: %d\n", gain);


	if (gain > 0xb) gain = 0xb;
	if (gain < 0) gain = 0;

	gain += gain << 4;

    es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, gain);
} 


// from ES8388 User guide

void dacInit() {
  es_write_reg(ES8388_ADDR, 0x08, 0x00); // set chip to slave
  delay(1);
  es_write_reg(ES8388_ADDR, 0x02, 0xF3); // Power Down DEM and STM
  delay(1);
  es_write_reg(ES8388_ADDR, 0x2B, 0x80); // Set same LRCLK
  delay(1);
  es_write_reg(ES8388_ADDR, 0x00, 0x05); // Set chip to play & record
  delay(1);
  es_write_reg(ES8388_ADDR, 0x01, 0x40); // Power up Analog and ibias
  delay(1);
  es_write_reg(ES8388_ADDR, 0x04, 0x3C); // Power up DAC / Analog Output

  es_write_reg(ES8388_ADDR, 0x17, 0x18); // Set DAC SFI (I2S 16 bit)
  //	es_write_reg(ES8388_ADDR, 0x18, 0x07); 		// Set MCLK/LRCLK 1024 for 22k
  es_write_reg(ES8388_ADDR, 0x18, 0x04); // Set MCLK/LRCLK 512 for 44k

  es_write_reg(ES8388_ADDR, 0x19, 0x32); // Unmute DAC
  es_write_reg(ES8388_ADDR, 0x1B, 0x00);

  es_write_reg(ES8388_ADDR, 0x26, 0x00); // Set Mixer for DAC Out
  es_write_reg(ES8388_ADDR, 0x27, 0xB8); // LDAC to LOUT
  es_write_reg(ES8388_ADDR, 0x28, 0x38);
  es_write_reg(ES8388_ADDR, 0x29, 0x38);
  es_write_reg(ES8388_ADDR, 0x2A, 0xB8); // RDAC to ROUT

  es_write_reg(ES8388_ADDR, 0x2E, 0x0); // set LOUT/ROUT Volume
  es_write_reg(ES8388_ADDR, 0x2F, 0x10);
  es_write_reg(ES8388_ADDR, 0x30, 0x0);
  es_write_reg(ES8388_ADDR, 0x31, 0x0);

  es_write_reg(ES8388_ADDR, 0x02, 0xAA); // Power up DEM and STM
}





/**
 * @brief Power Management
 *
 * @param mod:      if ES_POWER_CHIP, the whole chip including ADC and DAC is enabled
 * @param enable:   false to disable true to enable
 *
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
esp_err_t es8388_start(es_module_t mode)
{
    esp_err_t res = ESP_OK;
    uint8_t prev_data = 0, data = 0;
    es_read_reg(ES8388_DACCONTROL21, &prev_data);
    if (mode == ES_MODULE_LINE) {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x09); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2 by pass enable
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x50); // left DAC to left mixer enable  and  LIN signal to left mixer enable 0db  : bupass enable
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x50); // right DAC to right mixer enable  and  LIN signal to right mixer enable 0db : bupass enable
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0xC0); //enable adc
    } else {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80);   //enable dac
    }
    es_read_reg(ES8388_DACCONTROL21, &data);
    if (prev_data != data) {
        res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xF0);   //start state machine
        // res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x16);
        // res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
        res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00);   //start state machine
    }
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x00);   //power up adc and line in
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3c);   //power up dac and line out
        res |= es8388_set_voice_mute(false);
        ESP_LOGD(ES_TAG, "es8388_start default is mode:%d", mode);
    }

    return res;
}

/**
 * @brief Power Management
 *
 * @param mod:      if ES_POWER_CHIP, the whole chip including ADC and DAC is enabled
 * @param enable:   false to disable true to enable
 *
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
esp_err_t es8388_stop(es_module_t mode)
{
    esp_err_t res = ESP_OK;
    if (mode == ES_MODULE_LINE) {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80); //enable dac
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x00); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x90); // only left DAC to left mixer enable 0db
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x90); // only right DAC to right mixer enable 0db
        return res;
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x00);
        res |= es8388_set_voice_mute(true); //res |= Es8388SetAdcDacVolume(ES_MODULE_DAC, -96, 5);      // 0db
        //res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0xC0);  //power down dac and line out
    }
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        //res |= Es8388SetAdcDacVolume(ES_MODULE_ADC, -96, 5);      // 0db
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0xFF);  //power down adc and line in
    }
    if (mode == ES_MODULE_ADC_DAC) {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x9C);  //disable mclk
//        res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x00);
//        res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x58);
//        res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xF3);  //stop state machine
    }

    return res;
}


/**
 * @brief Config I2s clock in MSATER mode
 *
 * @param cfg.sclkDiv:      generate SCLK by dividing MCLK in MSATER mode
 * @param cfg.lclkDiv:      generate LCLK by dividing MCLK in MSATER mode
 *
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
esp_err_t es8388_i2s_config_clock(es_i2s_clock_t cfg)
{
    esp_err_t res = ESP_OK;
    res |= es_write_reg(ES8388_ADDR, ES8388_MASTERMODE, cfg.sclk_div);
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, cfg.lclk_div);  //ADCFsMode,singel SPEED,RATIO=256
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL2, cfg.lclk_div);  //ADCFsMode,singel SPEED,RATIO=256
    return res;
}

esp_err_t es8388_deinit(void)
{
    int res = 0;
    res = es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xFF);  //reset and stop es8388
    i2c_bus_delete(i2c_handle);
#ifdef CONFIG_ESP_LYRAT_V4_3_BOARD
    headphone_detect_deinit();
#endif

// MJB    audio_codec_volume_deinit(dac_vol_handle);
    return res;
}

/**
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
esp_err_t es8388_init(int sampleRate)
{
    int res = 0;
#ifdef CONFIG_ESP_LYRAT_V4_3_BOARD
    headphone_detect_init(get_headphone_detect_gpio());
#endif

	printf ("es8388_init ()\n");

    res = i2c_init(); // ESP32 in master mode

	printf ("i2c_init done ()\n");

/*
	dacInit();
	printf ("\ndacInit used instead of es8388_init code\n\n");
	return 0;
*/

	xinit ();
	printf ("\nxinit used instead of es8388_init code\n\n");
		
	return 0;	

    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x04);  // 0x04 mute/0x00 unmute&ramp;DAC unmute and  disabled digital volume control soft ramp
    /* Chip Control and Power Management */
    res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
    res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00); //normal all and power up all

    // Disable the internal DLL to improve 8K sample rate
    res |= es_write_reg(ES8388_ADDR, 0x35, 0xA0);
    res |= es_write_reg(ES8388_ADDR, 0x37, 0xD0);
    res |= es_write_reg(ES8388_ADDR, 0x39, 0xD0);

// MJB    res |= es_write_reg(ES8388_ADDR, ES8388_MASTERMODE, cfg->i2s_iface.mode); //CODEC IN I2S SLAVE MODE
    res |= es_write_reg(ES8388_ADDR, ES8388_MASTERMODE, AUDIO_HAL_MODE_SLAVE); //CODEC IN I2S SLAVE MODE

    /* dac */
    res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0xC0);  //disable DAC and disable Lout/Rout/1/2
    res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x12);  //Enfr=0,Play&Record Mode,(0x17-both of mic&paly)
//    res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0);  //LPVrefBuf=0,Pdn_ana=0
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, 0x18);//1a 0x18:16bit iis , 0x00:24
//    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL2, 0x02);  //DACFsMode,SINGLE SPEED; DACFsRatio,256
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL2, 0x04);  //DACFsMode,SINGLE SPEED; DACFsRatio,512
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x00); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x90); // only left DAC to left mixer enable 0db
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x90); // only right DAC to right mixer enable 0db
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80); // set internal ADC and DAC use the same LRCK clock, ADC LRCK as internal LRCK
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL23, 0x00); // vroi=0

    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL24, 0x1E); // Set L1 R1 L2 R2 volume. 0x00: -30dB, 0x1E: 0dB, 0x21: 3dB
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL25, 0x1E);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL26, 0);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL27, 0);
    // res |= es8388_set_adc_dac_volume(ES_MODULE_DAC, 0, 0);       // 0db

/*	MJB    
    int tmp = 0;
    if (AUDIO_HAL_DAC_OUTPUT_LINE2 == cfg->dac_output) {
        tmp = DAC_OUTPUT_LOUT1 | DAC_OUTPUT_ROUT1;
    } else if (AUDIO_HAL_DAC_OUTPUT_LINE1 == cfg->dac_output) {
        tmp = DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT2;
    } else {
        tmp = DAC_OUTPUT_LOUT1 | DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT1 | DAC_OUTPUT_ROUT2;
    }
*/
    int tmp = DAC_OUTPUT_LOUT1 | DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT1 | DAC_OUTPUT_ROUT2;    
    res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, tmp);  //0x3c Enable DAC and Enable Lout/Rout/1/2
    /* adc */
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0xFF);
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, 0xbb); // MIC Left and Right channel PGA gain
    tmp = 0;
/* MJB    
    if (AUDIO_HAL_ADC_INPUT_LINE1 == cfg->adc_input) {
        tmp = ADC_INPUT_LINPUT1_RINPUT1;
    } else if (AUDIO_HAL_ADC_INPUT_LINE2 == cfg->adc_input) {
        tmp = ADC_INPUT_LINPUT2_RINPUT2;
    } else {
        tmp = ADC_INPUT_DIFFERENCE;
    }
*/    
        tmp = ADC_INPUT_LINPUT1_RINPUT1;
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL2, tmp);  //0x00 LINSEL & RINSEL, LIN1/RIN1 as ADC Input; DSSEL,use one DS Reg11; DSR, LINPUT1-RINPUT1
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL3, 0x02);
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, 0x0d); // Left/Right data, Left/Right justified mode, Bits length, I2S format
//    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, 0x02);  //ADCFsMode,singel SPEED,RATIO=256

//	int v = (cfg->i2s_iface.samples == AUDIO_HAL_22K_SAMPLES)?0x7:0x4;
	int v = sampleRate == 22050 ? 0x7:0x4;
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, v);  //MJB 13/10/22 512 or 1024
	
//    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, 0x04);  //MJB 4/10/22 Try 512
 
    printf ("MJB setting sampling frequency ratio from config %d\n",v);

//    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, 0x01);  //ADCFsMode,singel SPEED,RATIO=192  WARNING MJB
    //ALC for Microphone
    
    
    res |= es8388_set_adc_dac_volume(ES_MODULE_ADC, 0, 0);      // 0db
    
    
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x09);    // Power on ADC, enable LIN&RIN, power off MICBIAS, and set int1lp to low power mode
  
/* MJB    
    // es8388 PA gpio_config
    gpio_config_t  io_conf;
    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = BIT64(get_pa_enable_gpio());
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
*/    
    /* enable es8388 PA  */
// MJB    es8388_pa_power(true);

// MJB    codec_dac_volume_config_t vol_cfg = ES8388_DAC_VOL_CFG_DEFAULT();
// MJB    dac_vol_handle = audio_codec_volume_init(&vol_cfg);
// MJB    ESP_LOGI(ES_TAG, "init,out:%02x, in:%02x", cfg->dac_output, cfg->adc_input);
    return res;
}

/**
 * @brief Configure ES8388 I2S format
 *
 * @param mode:           set ADC or DAC or all
 * @param bitPerSample:   see Es8388I2sFmt
 *
 * @return
 *     - (-1) Error
 *     - (0)  Success
 */
esp_err_t es8388_config_fmt(es_module_t mode, es_i2s_fmt_t fmt)
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        res = es_read_reg(ES8388_ADCCONTROL4, &reg);
        reg = reg & 0xfc;
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, reg | fmt);
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res = es_read_reg(ES8388_DACCONTROL1, &reg);
        reg = reg & 0xf9;
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, reg | (fmt << 1));
        
        printf ("\nes8388_config_fmt wrote %02x to ES8388_DACCONTROL1\n\n",reg | (fmt << 1));
        
    }
    return res;
}
// MJB
#if 0
/**
 * @brief Set voice volume
 *
 * @note Register values. 0xC0: -96 dB, 0x64: -50 dB, 0x00: 0 dB
 * @note Accuracy of gain is 0.5 dB
 *
 * @param volume: voice volume (0~100)
 *
 * @return
 *     - ESP_OK
 *     - ESP_FAIL
 */
esp_err_t es8388_set_voice_volume(int volume)
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;
    reg = audio_codec_get_dac_reg_value(dac_vol_handle, volume);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL5, reg);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL4, reg);
    ESP_LOGD(ES_TAG, "Set volume:%.2d reg_value:0x%.2x dB:%.1f", dac_vol_handle->user_volume, reg,
            audio_codec_cal_dac_volume(dac_vol_handle));
    return res;
}

esp_err_t es8388_get_voice_volume(int *volume)
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACCONTROL4, &reg);
    if (res == ESP_FAIL) {
        *volume = 0;
    } else {
        if (reg == dac_vol_handle->reg_value) {
            *volume = dac_vol_handle->user_volume;
        } else {
            *volume = 0;
            res = ESP_FAIL;
        }
    }
    ESP_LOGD(ES_TAG, "Get volume:%.2d reg_value:0x%.2x", *volume, reg);
    return res;
}
#endif

/**
 * @brief Configure ES8388 data sample bits
 *
 * @param mode:             set ADC or DAC or all
 * @param bitPerSample:   see BitsLength
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
esp_err_t es8388_set_bits_per_sample(es_module_t mode, es_bits_length_t bits_length)
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;
    int bits = (int)bits_length;

    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        res = es_read_reg(ES8388_ADCCONTROL4, &reg);
        reg = reg & 0xe3;
        res |=  es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, reg | (bits << 2));
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        res = es_read_reg(ES8388_DACCONTROL1, &reg);
        reg = reg & 0xc7;
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, reg | (bits << 3));
        
        printf ("\nes8388_set_bits_per_sample wrote %02x to ES8388_DACCONTROL1\n\n",reg | (bits << 3));
        
    }
    return res;
}

/**
 * @brief Configure ES8388 DAC mute or not. Basically you can use this function to mute the output or unmute
 *
 * @param enable: enable or disable
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
esp_err_t es8388_set_voice_mute(bool enable)
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACCONTROL3, &reg);
    reg = reg & 0xFB;
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, reg | (((int)enable) << 2));
    return res;
}

esp_err_t es8388_get_voice_mute(void)
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACCONTROL3, &reg);
    if (res == ESP_OK) {
        reg = (reg & 0x04) >> 2;
    }
    return res == ESP_OK ? reg : res;
}

/**
 * @param gain: Config DAC Output
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
esp_err_t es8388_config_dac_output(int output)
{
    esp_err_t res;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACPOWER, &reg);
    reg = reg & 0xc3;
    res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, reg | output);
    return res;
}

/**
 * @param gain: Config ADC input
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
esp_err_t es8388_config_adc_input(es_adc_input_t input)
{
    esp_err_t res;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_ADCCONTROL2, &reg);
    reg = reg & 0x0f;
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL2, reg | input);
    return res;
}

/**
 * @param gain: see es_mic_gain_t
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
esp_err_t es8388_set_mic_gain(es_mic_gain_t gain)
{
    esp_err_t res, gain_n;
    gain_n = (int)gain / 3;
    gain_n = (gain_n << 4) + gain_n;
    res = es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, gain_n); //MIC PGA
    return res;
}

int es8388_ctrl_state(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state)
{
    int res = 0;
    int es_mode_t = 0;
    switch (mode) {
        case AUDIO_HAL_CODEC_MODE_ENCODE:
            es_mode_t  = ES_MODULE_ADC;
            break;
        case AUDIO_HAL_CODEC_MODE_LINE_IN:
            es_mode_t  = ES_MODULE_LINE;
            break;
        case AUDIO_HAL_CODEC_MODE_DECODE:
            es_mode_t  = ES_MODULE_DAC;
            break;
        case AUDIO_HAL_CODEC_MODE_BOTH:
            es_mode_t  = ES_MODULE_ADC_DAC;
            break;
        default:
            es_mode_t = ES_MODULE_DAC;
            ESP_LOGW(ES_TAG, "Codec mode not support, default is decode mode");
            break;
    }
    if (AUDIO_HAL_CTRL_STOP == ctrl_state) {
        res = es8388_stop(es_mode_t);
    } else {
        res = es8388_start(es_mode_t);
        ESP_LOGD(ES_TAG, "start default is decode mode:%d", es_mode_t);
    }
    return res;
}

esp_err_t es8388_config_i2s(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t *iface)
{
    esp_err_t res = ESP_OK;
    int tmp = 0;
    res |= es8388_config_fmt(ES_MODULE_ADC_DAC, iface->fmt);
    if (iface->bits == AUDIO_HAL_BIT_LENGTH_16BITS) {
        tmp = BIT_LENGTH_16BITS;
    } else if (iface->bits == AUDIO_HAL_BIT_LENGTH_24BITS) {
        tmp = BIT_LENGTH_24BITS;
    } else {
        tmp = BIT_LENGTH_32BITS;
    }
    res |= es8388_set_bits_per_sample(ES_MODULE_ADC_DAC, tmp);
    return res;
}

/* MJB
void es8388_pa_power(bool enable)
{
    if (enable) {
        gpio_set_level(get_pa_enable_gpio(), 1);
    } else {
        gpio_set_level(get_pa_enable_gpio(), 0);
    }
}
*/



//********************************************************************************
// This is code from https://github.com/thaaraak/ESP32-ES8388/blob/master/main/main.c
//*********************************************************************************

typedef enum {
    I2S_NORMAL = 0,  /*!< set normal I2S format */
    I2S_LEFT,        /*!< set all left format */
    I2S_RIGHT,       /*!< set all right format */
    I2S_DSP,         /*!< set dsp/pcm format */
} es_format_t;

esp_err_t xes8388_init( es_dac_output_t output, es_adc_input_t input )
{
    int res = 0;

    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x04);  // 0x04 mute/0x00 unmute&ramp;DAC unmute and  disabled digital volume control soft ramp

    res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
    res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00); //normal all and power up all
    res |= es_write_reg(ES8388_ADDR, ES8388_MASTERMODE, ES_MODE_SLAVE ); //CODEC IN I2S SLAVE MODE

    res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0xC0);  //disable DAC and disable Lout/Rout/1/2
    res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x12);  //Enfr=0,Play&Record Mode,(0x17-both of mic&paly)
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, 0x1A);//1a 0x18:16bit iis , 0x00:24
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL2, 0x02);  //DACFsMode,SINGLE SPEED; DACFsRatio,256
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x00); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x90); // only left DAC to left mixer enable 0db
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x90); // only right DAC to right mixer enable 0db
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80); //set internal ADC and DAC use the same LRCK clock, ADC LRCK as internal LRCK
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL23, 0x00);   //vroi=0
    res |= es8388_set_adc_dac_volume(ES_MODULE_DAC, 0, 0);          // 0db

    ESP_LOGE(ES_TAG, "Setting DAC Output: %02x", output );
    res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, output );
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0xFF);
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, 0xbb); // MIC Left and Right channel PGA gain


    ESP_LOGE(ES_TAG, "Setting ADC Input: %02x", input );
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL2, input);

    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL3, 0x02);
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, 0x0d); // Left/Right data, Left/Right justified mode, Bits length, I2S format
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, 0x02);  //ADCFsMode,singel SPEED,RATIO=256
    //ALC for Microphone
    res |= es8388_set_adc_dac_volume(ES_MODULE_ADC, 0, 0);      // 0db
    res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x09); //Power on ADC, Enable LIN&RIN, Power off MICBIAS, set int1lp to low power mode

    return res;
}

// This function sets the I2S format which can be one of
//		I2S_NORMAL
//		I2S_LEFT		Left Justified
//		I2S_RIGHT,      Right Justified
//		I2S_DSP,        dsp/pcm format
//
// and the bits per sample which must be one of
//		BIT_LENGTH_16BITS
//		BIT_LENGTH_18BITS
//		BIT_LENGTH_20BITS
//		BIT_LENGTH_24BITS
//		BIT_LENGTH_32BITS
//
// Note the above must match the ESP-IDF I2S configuration which is set separately

esp_err_t xes8388_config_i2s( es_bits_length_t bits_length, es_module_t mode, es_format_t fmt )
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;

    // Set the Format
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        printf( "Setting I2S ADC Format\n");
        res = es_read_reg(ES8388_ADCCONTROL4, &reg);
        reg = reg & 0xfc;
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, reg | fmt);
    }
//    printf ("xes8388_config_i2s hacked - do not touch DACCONTROL1\n");
    
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
		fmt = 1;
        printf( "Setting I2S DAC Format %d\n",fmt);
        res = es_read_reg(ES8388_DACCONTROL1, &reg);
        reg = reg & 0xf9;
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, reg | (fmt << 1));
    }


    // Set the Sample bits length
    int bits = (int)bits_length;
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        printf( "Setting I2S ADC Bits: %d\n", bits);
        res = es_read_reg(ES8388_ADCCONTROL4, &reg);
        reg = reg & 0xe3;
        res |=  es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, reg | (bits << 2));
    }
    
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        ESP_LOGE(ES_TAG, "Setting I2S DAC Bits: %d\n", bits);
        res = es_read_reg(ES8388_DACCONTROL1, &reg);
        reg = reg & 0xc7;
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, reg | (bits << 3));
    }

    es_read_reg(ES8388_DACCONTROL1, &reg);
    printf ("xes_config_i2s() final value %02x\n",reg);
            
    return res;
}


esp_err_t xes8388_set_voice_mute(bool enable)
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;
    res = es_read_reg(ES8388_DACCONTROL3, &reg);
    reg = reg & 0xFB;
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, reg | (((int)enable) << 2));
    return res;
}

esp_err_t xes8388_start(es_module_t mode)
{
    esp_err_t res = ESP_OK;
    uint8_t prev_data = 0, data = 0;
    es_read_reg(ES8388_DACCONTROL21, &prev_data);
    if (mode == ES_MODULE_LINE) {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x09); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2 by pass enable
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x50); // left DAC to left mixer enable  and  LIN signal to left mixer enable 0db  : bupass enable
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x50); // right DAC to right mixer enable  and  LIN signal to right mixer enable 0db : bupass enable
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0xC0); //enable adc
    } else {
        res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80);   //enable dac
    }
    es_read_reg(ES8388_DACCONTROL21, &data);
    if (prev_data != data) {
    	printf( "Resetting State Machine\n");

        res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xF0);   //start state machine
        // res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x16);
        // res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
        res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00);   //start state machine
    }
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
    	printf( "Powering up ADC\n");
        res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x00);   //power up adc and line in
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
    	printf( "Powering up DAC\n");
        res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3c);   //power up dac and line out
        res |= es8388_set_voice_mute(false);
    }

    return res;
}


esp_err_t xes8388_set_voice_volume(int volume)
{
    esp_err_t res = ESP_OK;
    if (volume < 0)
        volume = 0;
    else if (volume > 100)
        volume = 100;
    volume /= 3;
    res = es_write_reg(ES8388_ADDR, ES8388_DACCONTROL24, volume);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL25, volume);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL26, volume);
    res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL27, volume);
    return res;
}


void xes8388_config()
{
    // Input/Output Modes
    //
    //	es_dac_output_t output = DAC_OUTPUT_LOUT1 | DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT1 | DAC_OUTPUT_ROUT2;
    //	es_adc_input_t input = ADC_INPUT_LINPUT1_RINPUT1;
    // 	es_adc_input_t input = ADC_INPUT_LINPUT2_RINPUT2;

    es_dac_output_t output = DAC_OUTPUT_LOUT1 | DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT1 | DAC_OUTPUT_ROUT2;
	//es_dac_output_t output = DAC_OUTPUT_LOUT1  | DAC_OUTPUT_ROUT1;
	//es_dac_output_t output = DAC_OUTPUT_LOUT2  | DAC_OUTPUT_ROUT2;

    //es_dac_output_t output = 0;
	es_adc_input_t input = ADC_INPUT_LINPUT1_RINPUT1;

    xes8388_init( output, input );

    // Modes Available
    //
    //	es_mode_t  = ES_MODULE_ADC;
    //	es_mode_t  = ES_MODULE_LINE;
    //	es_mode_t  = ES_MODULE_DAC;
    //	es_mode_t  = ES_MODULE_ADC_DAC;

    es_bits_length_t bits_length = BIT_LENGTH_16BITS;
    es_module_t module = ES_MODULE_DAC;
    es_format_t fmt = I2S_NORMAL;

    xes8388_config_i2s( bits_length, ES_MODULE_ADC_DAC, fmt );
    xes8388_set_voice_volume( 50 );
    xes8388_start( module );

}


#if 0
static void setup_sine_waves16( int amplitude )
{
	double sin_float;

    size_t i2s_bytes_write = 0;

    //printf("\r\nFree mem=%d, written data=%d\n", esp_get_free_heap_size(), BUF_SAMPLES*2 );

    for( int pos = 0; pos < BUF_SAMPLES; pos += 2 )
    {
        sin_float = amplitude * sin( pos/2 * 2 * PI / SAMPLE_PER_CYCLE);

        int lval = sin_float;
        int rval = sin_float;

        txBuf[pos] = lval&0xFFFF;
        txBuf[pos+1] = rval&0xFFFF;

        //printf( "%d  %04x:%04x\n", lval, txBuf[pos],txBuf[pos+1] );

    }
}
#endif

void xinit(void)
{
//	i2c_master_init();
	xes8388_config();

#if 0
	es8388_read_all();


	i2s_init();

    size_t i2s_bytes_write = 0;

    int amplitude = 8000;
    int start_dir = 50;
    int dir = start_dir;

    while (1) {

/*    	amplitude -= dir;
    	if ( amplitude <= start_dir || amplitude >= 15000 )
    		dir *= -1;
*/
    	setup_sine_waves16( amplitude );

    	//i2s_write(I2S_NUM, txBuf, BUF_SAMPLES*2, &i2s_bytes_write, -1);
    	i2s_write(I2S_NUM, txBuf, BUF_SAMPLES*2, &i2s_bytes_write, -1);
    	//printf( "Bytes: %d\n", i2s_bytes_write );
    	//vTaskDelay(10/portTICK_RATE_MS);

    }
 #endif   
}





