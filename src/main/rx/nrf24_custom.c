/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// this file is copied with modifications from bradwii for jd385
// see https://github.com/hackocopter/bradwii-jd385

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"
#include "flight/position.h"

#ifdef USE_RX_V202

#include "build/build_config.h"
#include "build/debug.h"

#include "common/utils.h"

#include "pg/rx.h"

#include "drivers/io.h"
#include "drivers/rx/rx_nrf24l01.h"
#include "drivers/time.h"

#include "rx/rx.h"
#include "rx/rx_spi.h"
#include "rx/rx_spi_common.h"
#include "drivers/rx/rx_xn297.h"
#include "rx/nrf24_v202.h"
#include "sensors/battery.h"

/*
 * Custom Houston protocol
 */
#define V2X2_DEBUG

/*
    RC_SPI_ROLL = 0,
    RC_SPI_PITCH,
    RC_SPI_THROTTLE,
    RC_SPI_YAW,
    RC_SPI_AUX1,
    RC_SPI_AUX2,
    RC_SPI_AUX3,
*/
#define V2X2_RC_CHANNEL_COUNT 6

#define CRC_LEN 2

#define COMM_PING 1    /* keepalive */
#define ESC_CAL_1 2    /* esc calibration, first step max. */
#define ESC_CAL_2 3    /* esc calibration, second step min. */
#define MANUAL_MOTOR 4 /* manual motor control 1-4 in percentages, 5-all pct. */
#define PID_CAL 6      /* PID settings */
#define IMU_ORIENT 7   /* to save initial imu positions */
#define FLIGHT 10      /* for flight */

// flight value
#define FLIGHT_ON 1  /* arm */
#define FLIGHT_OFF 2 /* shutdown */
#define FLIGHT_CMD 3 /* sticks */
#define FLIGHT_PID 4 /* stabilisation algo toggle on */

// telemetry codes
#define TEL_FLIGHT_BIT 0
#define TEL_PID_BIT_1 1
#define TEL_PID_BIT_2 2
// #define TEL_ACCEL_CAL_BIT 3

// pid modes
// #define PID_OFF 0
// #define PID_LEVEL 1
#define PID_ANGLE 2 // only supported mode for now
// #define PID_ACRO 3

#define NRF_CHANNEL 0x50
#define NRF_PAYLOAD_SIZE 8 // TODO: check this out

#define RX_TX_ADDR_LEN 5
#define TELEMETRY_SIZE 32

const uint8_t rx_addr[RX_TX_ADDR_LEN] = {0xE2, 0xF0, 0xF0, 0xF0, 0xF0}; // Houston module
const uint8_t tx_addr[RX_TX_ADDR_LEN] = {0x32, 0x76, 0x72, 0x65, 0x73};

// extern uint16_t rxSpiRcData[];

typedef struct
{
    uint8_t op;
    uint8_t value;
    uint8_t aux1;
    uint8_t aux2;
    uint8_t throttle;
    uint8_t yaw;
    uint8_t pitch;
    uint8_t roll;
} Command;

/* 31 bytes, but it is 36, padding it */
typedef struct
{
    float pitch; // 4
    float roll; // 4
    float yaw; // 4

    // int16_t altitude; // no memory for BME sensor
    // int8_t temperature; // no memory for BME sensor

    uint16_t imu_counts; // 2
    uint16_t pid_counts; // 2

    uint16_t thr_tr; // 2
    uint16_t thr_br; // 2

    uint16_t thr_bl; // 2
    uint16_t thr_tl; // 2

    int16_t pid_pitch; // 2
    int16_t pid_roll; // 2

    int16_t pid_yaw; // 2
    uint8_t basic; // 1
    // padding 1
} Telemetry;

uint8_t new_telemetry[TELEMETRY_SIZE];

//static Command command;
static Telemetry telemetry;

static bool flight = false;
static bool init_buffer = false;
uint8_t count = 0; // each 10 one ack

static uint16_t convertToPwmUnsigned(uint8_t val)
{
    uint32_t ret = val;
    ret = ret * (PWM_RANGE_MAX - PWM_RANGE_MIN) / UINT8_MAX + PWM_RANGE_MIN;
    return (uint16_t)ret;
}

/*static uint16_t convertToPwmSigned(uint8_t val)
{
    int32_t ret = val & 0x7f;
    ret = (ret * (PWM_RANGE_MAX - PWM_RANGE_MIN)) / (2 * INT8_MAX);
    if (val & 0x80)
    { // sign bit set
        ret = -ret;
    }
    return (uint16_t)(PWM_RANGE_MIDDLE + ret);
}*/

/* important function to convert the data
 */
void v202Nrf24SetRcDataFromPayload(uint16_t *rcData, const uint8_t *packet)
{

    if (init_buffer == false) {
        // clean buffer on init
        for(int i = 0; i < V2X2_RC_CHANNEL_COUNT; i++) {
            rcData[i] = 1000;
        }

        init_buffer = true;
    }
    Command* command = (Command*) packet;        
    telemetry.basic = 1;
    telemetry.imu_counts = getBatteryVoltage();
    telemetry.pid_pitch = (int16_t) getEstimatedAltitudeCm();
    
    if (command->op == FLIGHT) {
        /* don't mess the values with PID_CAL, etc. */
        rcData[RC_SPI_THROTTLE] = convertToPwmUnsigned(command->throttle);
        rcData[RC_SPI_YAW] = convertToPwmUnsigned(command->yaw);   // rudder
        rcData[RC_SPI_PITCH] = convertToPwmUnsigned(command->pitch); // elevator
        rcData[RC_SPI_ROLL] = convertToPwmUnsigned(command->roll);  // aileron     
    }
    
    if (command->op == FLIGHT) {
        if(command->value == FLIGHT_CMD) {            
            // We are flying                
        } else if (command->value == FLIGHT_ON) {            
            /* ARM */
            if(flight) {
                // disable prearm
                rcData[RC_SPI_AUX1] = 1000;
            }
            rcData[RC_SPI_AUX2] = 1100;
            flight = true;
        } else if (command->value == FLIGHT_OFF) {
            /* DISARM */
            rcData[RC_SPI_THROTTLE] = 1000; // just in case
            rcData[RC_SPI_AUX1] = 1000;
            rcData[RC_SPI_AUX2] = 1000;
            flight = false;            
        }
    }        
    else if (command->op == PID_CAL) 
    {
        /* we are using this as a prearm */
        rcData[RC_SPI_AUX1] = 1100;
        rcData[RC_SPI_AUX2] = 1000; 

    }
    else if (command->op == COMM_PING)
    {
        // nothing here
    }

    /* has to be valid, so reset this.
    for(int i = 0; i < V2X2_RC_CHANNEL_COUNT; i++) {
        if (rcData[i] < 1000 || rcData[i] > 2000) {
            rcData[i] = 1000;
        }
    } */
}

/* called by the scheduler */
rx_spi_received_e v202Nrf24DataReceived(uint8_t *payload)
{

    rx_spi_received_e ret = RX_SPI_RECEIVED_NONE;

    if (NRF24L01_ReadPayloadIfAvailable(payload, NRF_PAYLOAD_SIZE)) {
        
        NRF24L01_WriteReg(NRF24L01_07_STATUS, BIT(NRF24L01_07_STATUS_RX_DR)); // clear the RX_DR flag        

        Command* command = (Command*) payload;    

        #ifdef V2X2_DEBUG
            debug[0] = command->op;
            debug[1] = command->value;
            debug[2] = command->throttle;
            debug[3] = sizeof(telemetry);
        #endif

        /* zeros are not valid for Betaflight */
        if (command->op != 0x00)
        {

            if (count % 5 == 0) {       
                NRF24L01_WriteReg(NRF24L01_07_STATUS, BIT(NRF24L01_07_STATUS_MAX_RT));
                NRF24L01_WriteAckPayload((uint8_t *) &telemetry, sizeof(telemetry), NRF24L01_PIPE1);
            }
            count = count + 1;

            ret = RX_SPI_RECEIVED_DATA;        
        }

    }    

    return ret;
}

static void v202Nrf24Setup(rx_spi_protocol_e protocol)
{


    NRF24L01_Initialize(BIT(NRF24L01_00_CONFIG_EN_CRC) | BIT(NRF24L01_00_CONFIG_CRCO)); // 2-byte CRC    
    
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, NRF24L01_03_SETUP_AW_5BYTES); // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, (uint8_t) (NRF24L01_04_SETUP_RETR_ARD_2250us << 4) | NRF24L01_04_SETUP_RETR_ARC_10);

    if (protocol == RX_SPI_NRF24_V202_250K)
    {
        NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, NRF24L01_06_RF_SETUP_RF_DR_250Kbps | NRF24L01_06_RF_SETUP_RF_PWR_n6dbm);
    }
    else
    {
        NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, NRF24L01_06_RF_SETUP_RF_DR_1Mbps | NRF24L01_06_RF_SETUP_RF_PWR_n12dbm);
    }

    uint8_t before_toggle = NRF24L01_ReadReg(NRF24L01_1D_FEATURE);
    NRF24L01_Activate(0x73);
    uint8_t after_toggle = NRF24L01_ReadReg(NRF24L01_1D_FEATURE);
    bool _is_p_variant = before_toggle == after_toggle;

    if (after_toggle) {
        if (_is_p_variant) {
            // module did not experience power-on-reset (#401)
            NRF24L01_Activate(0x73);
        }
        // allow use of multicast parameter and dynamic payloads by default
        NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0);
    }
    
    // payload size
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, NRF_PAYLOAD_SIZE);
    NRF24L01_WriteReg(NRF24L01_12_RX_PW_P1, NRF_PAYLOAD_SIZE);
    NRF24L01_WriteReg(NRF24L01_13_RX_PW_P2, NRF_PAYLOAD_SIZE);
    NRF24L01_WriteReg(NRF24L01_14_RX_PW_P3, NRF_PAYLOAD_SIZE);
    NRF24L01_WriteReg(NRF24L01_15_RX_PW_P4, NRF_PAYLOAD_SIZE);
    NRF24L01_WriteReg(NRF24L01_16_RX_PW_P5, NRF_PAYLOAD_SIZE);
    
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, NRF_CHANNEL);    
        
    NRF24L01_WriteReg(NRF24L01_07_STATUS, BIT(NRF24L01_07_STATUS_RX_DR) | BIT(NRF24L01_07_STATUS_TX_DS) | BIT(NRF24L01_07_STATUS_MAX_RT));     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_17_FIFO_STATUS, 0x00);

    // OLD: NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, BIT(NRF24L01_02_EN_RXADDR_ERX_P1) | BIT(NRF24L01_02_EN_RXADDR_ERX_P2));

    // activate ack and dyn payloads
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, NRF24L01_01_EN_AA_ALL_PIPES); // auto acknowledgment all pipes
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, NRF24L01_1C_DYNPD_ALL_PIPES); // enable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, NRF24L01_ReadReg(NRF24L01_1D_FEATURE) | BIT(NRF24L01_1D_FEATURE_EN_ACK_PAY) | BIT(NRF24L01_1D_FEATURE_EN_DPL));  
    // NRF24L01_WriteReg(NRF24L01_1C_DYNPD, NRF24L01_ReadReg(NRF24L01_1C_DYNPD) | BIT(NRF24L01_1C_DYNPD_DPL_P1) | BIT(NRF24L01_1C_DYNPD_DPL_P0));  
    // set pipe 1 for reading    
    NRF24L01_WriteRegisterMulti(NRF24L01_0B_RX_ADDR_P1, rx_addr, RX_TX_ADDR_LEN);
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, NRF24L01_ReadReg(NRF24L01_02_EN_RXADDR) | BIT(NRF24L01_02_EN_RXADDR_ERX_P1));
    
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, tx_addr, RX_TX_ADDR_LEN);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, tx_addr, RX_TX_ADDR_LEN); 

    NRF24L01_FlushTx();
    NRF24L01_FlushRx();

    NRF24L01_SetRxMode(); // enter receive mode to start listening for packets

    // close reading pipe on 0
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, NRF24L01_ReadReg(NRF24L01_02_EN_RXADDR) & ~BIT(NRF24L01_02_EN_RXADDR_ERX_P0));

    NRF24L01_WriteReg(NRF24L01_07_STATUS, BIT(NRF24L01_07_STATUS_MAX_RT));
    NRF24L01_WriteAckPayload((uint8_t *) &telemetry, sizeof(telemetry), NRF24L01_PIPE1);


   
}

bool v202Nrf24Init(const struct rxSpiConfig_s *rxSpiConfig, struct rxRuntimeState_s *rxRuntimeState, rxSpiExtiConfig_t *extiConfig)
{
    UNUSED(extiConfig);    

    rxRuntimeState->channelCount = V2X2_RC_CHANNEL_COUNT;
    v202Nrf24Setup((rx_spi_protocol_e)rxSpiConfig->rx_spi_protocol);    
    return true;
}
#endif
