/**
  ******************************************************************************
  * @file           : ndef_poling.c
  * @brief          : Ndef Polling file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/*! \file
 *
 *  \author
 *
 *  \brief Demo application
 *
 *  This demo shows how to poll for several types of NFC cards/devices and how
 *  to exchange data with these devices, using the RFAL library.
 *
 *  This demo does not fully implement the activities according to the standards,
 *  it performs the required to communicate with a card/device and retrieve
 *  its UID. Also blocking methods are used for data exchange which may lead to
 *  long periods of blocking CPU/MCU.
 *  For standard compliant example please refer to the Examples provided
 *  with the RFAL library.
 *
 */

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include "demo.h"
#include "utils.h"
#include "rfal_nfc.h"
#include "rfal_t2t.h"
#include "main.h"

#if RFAL_SUPPORT_CE && RFAL_FEATURE_LISTEN_MODE
#include "demo_ce.h"
#endif /* RFAL_FEATURE_LISTEN_MODE */

/** @addtogroup X-CUBE-NFC6_Applications
 *  @{
 */

/** @addtogroup PollingTagDetect
 *  @{
 */

/** @addtogroup PTD_Demo
 * @{
 */

/*
******************************************************************************
* GLOBAL DEFINES
******************************************************************************
*/
/** @defgroup PTD_Demo_Private_Define
 * @{
 */
/* Definition of possible states the demo state machine could have */
#define DEMO_ST_NOTINIT               0     /*!< Demo State:  Not initialized        */
#define DEMO_ST_START_DISCOVERY       1     /*!< Demo State:  Start Discovery        */
#define DEMO_ST_DISCOVERY             2     /*!< Demo State:  Discovery              */

#define DEMO_NFCV_BLOCK_LEN           4     /*!< NFCV Block len                      */

#define DEMO_NFCV_USE_SELECT_MODE     false /*!< NFCV demonstrate select mode        */
#define DEMO_NFCV_WRITE_TAG           false /*!< NFCV demonstrate Write Single Block */

/**
  * @}
  */
/*
 ******************************************************************************
 * GLOBAL MACROS
 ******************************************************************************
 */

/*
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */
/** @defgroup PTD_Demo_Private_Variables
 * @{
 */


/* P2P communication data */
static uint8_t NFCID3[] = {0x01, 0xFE, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
static uint8_t GB[] = {0x46, 0x66, 0x6d, 0x01, 0x01, 0x11, 0x02, 0x02, 0x07, 0x80, 0x03, 0x02, 0x00, 0x03, 0x04, 0x01, 0x32, 0x07, 0x01, 0x03};

/* APDUs communication data */
#if RFAL_FEATURE_ISO_DEP_POLL
static uint8_t ndefSelectApp[] = { 0x00, 0xA4, 0x04, 0x00, 0x07, 0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01, 0x00 };
static uint8_t ccSelectFile[] = { 0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x03};
static uint8_t readBinary[] = { 0x00, 0xB0, 0x00, 0x00, 0x0F };

/* For a Payment application a Select PPSE would be needed:
   ppseSelectApp[] = { 0x00, 0xA4, 0x04, 0x00, 0x0E, 0x32, 0x50, 0x41, 0x59, 0x2E, 0x53, 0x59, 0x53, 0x2E, 0x44, 0x44, 0x46, 0x30, 0x31, 0x00 } */
#endif /* RFAL_FEATURE_ISO_DEP_POLL */

#if RFAL_FEATURE_NFC_DEP
/* P2P communication data */
static uint8_t ndefInit[] = {0x05, 0x20, 0x06, 0x0F, 0x75, 0x72, 0x6E, 0x3A, 0x6E, 0x66, 0x63, 0x3A, 0x73, 0x6E, 0x3A, 0x73, 0x6E, 0x65, 0x70, 0x02, 0x02, 0x07, 0x80, 0x05, 0x01, 0x02};
#endif /* RFAL_FEATURE_NFC_DEP */

#if RFAL_SUPPORT_CE && RFAL_FEATURE_LISTEN_MODE
#if RFAL_SUPPORT_MODE_LISTEN_NFCA
/* NFC-A CE config */
/* 4-byte UIDs with first byte 0x08 would need random number for the subsequent 3 bytes.
 * 4-byte UIDs with first byte 0x*F are Fixed number, not unique, use for this demo
 * 7-byte UIDs need a manufacturer ID and need to assure uniqueness of the rest.*/
static uint8_t ceNFCA_NFCID[]     = {0x5F, 'S', 'T', 'M'};    /* =_STM, 5F 53 54 4D NFCID1 / UID (4 bytes) */
static uint8_t ceNFCA_SENS_RES[]  = {0x02, 0x00};             /* SENS_RES / ATQA for 4-byte UID            */
static uint8_t ceNFCA_SEL_RES     = 0x20;                     /* SEL_RES / SAK                             */
#endif /* RFAL_SUPPORT_MODE_LISTEN_NFCA */

static uint8_t ceNFCF_nfcid2[]     = {0x02, 0xFE, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66};

#if RFAL_SUPPORT_MODE_LISTEN_NFCF
  /* NFC-F CE config */
static uint8_t ceNFCF_SC[]         = {0x12, 0xFC};
static uint8_t ceNFCF_SENSF_RES[]  = {0x01,                                                   /* SENSF_RES                                */
                                  0x02, 0xFE, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,             /* NFCID2                                   */
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x7F, 0x00,             /* PAD0, PAD01, MRTIcheck, MRTIupdate, PAD2 */
                                  0x00, 0x00 };                                               /* RD                                       */
#endif /* RFAL_SUPPORT_MODE_LISTEN_NFCF */
#endif /* RFAL_SUPPORT_CE && RFAL_FEATURE_LISTEN_MODE */

/*
 ******************************************************************************
 * LOCAL VARIABLES
 ******************************************************************************
 */

static rfalNfcDiscoverParam discParam;
static uint8_t              state = DEMO_ST_NOTINIT;
static bool                 multiSel;

/**
  * @}
  */

/*
******************************************************************************
* LOCAL FUNCTION PROTOTYPES
******************************************************************************
*/

static void demoP2P( rfalNfcDevice *nfcDev );
static void demoAPDU( void );
static void demoNfcv( rfalNfcvListenDevice *nfcvDev );
static void demoNfcf( rfalNfcfListenDevice *nfcfDev );
static void demoT2t( void );
static void demoCE( rfalNfcDevice *nfcDev );
static void demoNotif( rfalNfcState st );


/** @defgroup PTD_Demo_Private_Functions
 * @{
 */
/*!
 *****************************************************************************
 * \brief Demo Notification
 *
 *  This function receives the event notifications from RFAL
 *****************************************************************************
 */
static void demoNotif( rfalNfcState st )
{
    uint8_t       devCnt;
    rfalNfcDevice *dev;

    if( st == RFAL_NFC_STATE_WAKEUP_MODE )
    {
        platformLog("Wake Up mode started \r\n");
    }
    else if( st == RFAL_NFC_STATE_POLL_TECHDETECT )
    {
        if( discParam.wakeupEnabled )
        {
            platformLog("Wake Up mode terminated. Polling for devices \r\n");
        }
    }
    else if( st == RFAL_NFC_STATE_POLL_SELECT )
    {
        /* Check if in case of multiple devices, selection is already attempted */
        if( (!multiSel) )
        {
            multiSel = true;
            /* Multiple devices were found, activate first of them */
            rfalNfcGetDevicesFound( &dev, &devCnt );
            rfalNfcSelect( 0 );

            platformLog("Multiple Tags detected: %d \r\n", devCnt);
        }
        else
        {
            rfalNfcDeactivate( RFAL_NFC_DEACTIVATE_DISCOVERY );
        }
    }
    else if( st == RFAL_NFC_STATE_START_DISCOVERY )
    {
        /* Clear mutiple device selection flag */
    	if (devCnt>0)
    	{
    		rfalNfcSelect(1);
    	}

        multiSel = false;
    }
}

/*!
 *****************************************************************************
 * \brief Demo Ini
 *
 *  This function Initializes the required layers for the demo
 *
 * \return true  : Initialization ok
 * \return false : Initialization failed
 *****************************************************************************
 */
bool demoIni( void )
{
    ReturnCode err;

    err = rfalNfcInitialize();
    if( err == RFAL_ERR_NONE )
    {
        rfalNfcDefaultDiscParams( &discParam );

        discParam.devLimit      = 1U;

        ST_MEMCPY( &discParam.nfcid3, NFCID3, sizeof(NFCID3) );
        ST_MEMCPY( &discParam.GB, GB, sizeof(GB) );
        discParam.GBLen         = sizeof(GB);
        discParam.p2pNfcaPrio   = true;

        discParam.notifyCb             = demoNotif;
        discParam.totalDuration        = 1000U;
        discParam.techs2Find           = RFAL_NFC_TECH_NONE;          /* For the demo, enable the NFC Technlogies based on RFAL Feature switches */

#if RFAL_FEATURE_NFCA
        discParam.techs2Find          |= RFAL_NFC_POLL_TECH_A;
#endif /* RFAL_FEATURE_NFCA */

#if RFAL_FEATURE_NFCB
        discParam.techs2Find          |= RFAL_NFC_POLL_TECH_B;
#endif /* RFAL_FEATURE_NFCB */

#if RFAL_FEATURE_NFCF
        discParam.techs2Find          |= RFAL_NFC_POLL_TECH_F;
#endif /* RFAL_FEATURE_NFCF */

#if RFAL_FEATURE_NFCV
        discParam.techs2Find          |= RFAL_NFC_POLL_TECH_V;
#endif /* RFAL_FEATURE_NFCV */

#if RFAL_FEATURE_ST25TB
        discParam.techs2Find          |= RFAL_NFC_POLL_TECH_ST25TB;
#endif /* RFAL_FEATURE_ST25TB */

#if ST25R95
        discParam.isoDepFS           = RFAL_ISODEP_FSXI_128;          /* ST25R95 cannot support 256 bytes of data block */
#endif /* ST25R95 */

//#if RFAL_SUPPORT_MODE_POLL_ACTIVE_P2P && RFAL_FEATURE_NFC_DEP
//        discParam.techs2Find |= RFAL_NFC_POLL_TECH_AP2P;
//#endif /* RFAL_SUPPORT_MODE_POLL_ACTIVE_P2P && RFAL_FEATURE_NFC_DEP */

//#if RFAL_SUPPORT_MODE_LISTEN_ACTIVE_P2P && RFAL_FEATURE_NFC_DEP && RFAL_FEATURE_LISTEN_MODE
//        discParam.techs2Find |= RFAL_NFC_LISTEN_TECH_AP2P;
//#endif /* RFAL_SUPPORT_MODE_LISTEN_ACTIVE_P2P && RFAL_FEATURE_NFC_DEP && RFAL_FEATURE_LISTEN_MODE */

#if DEMO_CARD_EMULATION_ONLY
        discParam.totalDuration        = 60000U;              /* 60 seconds */
        discParam.techs2Find           = RFAL_NFC_TECH_NONE;  /* Overwrite any previous poller modes */
#endif /* DEMO_CARD_EMULATION_ONLY */

#if RFAL_SUPPORT_CE && RFAL_FEATURE_LISTEN_MODE
        demoCeInit( ceNFCF_nfcid2 );

#if RFAL_SUPPORT_MODE_LISTEN_NFCA
        /* Set configuration for NFC-A CE */
        ST_MEMCPY( discParam.lmConfigPA.SENS_RES, ceNFCA_SENS_RES, RFAL_LM_SENS_RES_LEN );     /* Set SENS_RES / ATQA */
        ST_MEMCPY( discParam.lmConfigPA.nfcid, ceNFCA_NFCID, RFAL_LM_NFCID_LEN_04 );           /* Set NFCID / UID */
        discParam.lmConfigPA.nfcidLen = RFAL_LM_NFCID_LEN_04;                                  /* Set NFCID length to 7 bytes */
        discParam.lmConfigPA.SEL_RES  = ceNFCA_SEL_RES;                                        /* Set SEL_RES / SAK */

        //discParam.techs2Find |= RFAL_NFC_LISTEN_TECH_A;
#endif /* RFAL_SUPPORT_MODE_LISTEN_NFCA */

#if RFAL_SUPPORT_MODE_LISTEN_NFCF
        /* Set configuration for NFC-F CE */
        ST_MEMCPY( discParam.lmConfigPF.SC, ceNFCF_SC, RFAL_LM_SENSF_SC_LEN );                 /* Set System Code */
        ST_MEMCPY( &ceNFCF_SENSF_RES[RFAL_NFCF_CMD_LEN], ceNFCF_nfcid2, RFAL_NFCID2_LEN );     /* Load NFCID2 on SENSF_RES */
        ST_MEMCPY( discParam.lmConfigPF.SENSF_RES, ceNFCF_SENSF_RES, RFAL_LM_SENSF_RES_LEN );  /* Set SENSF_RES / Poll Response */

        //discParam.techs2Find |= RFAL_NFC_LISTEN_TECH_F;
#endif /* RFAL_SUPPORT_MODE_LISTEN_NFCF */
#endif /* RFAL_SUPPORT_CE && RFAL_FEATURE_LISTEN_MODE */

        /* Check for valid configuration by calling Discover once */
        err = rfalNfcDiscover( &discParam );
        rfalNfcDeactivate( RFAL_NFC_DEACTIVATE_IDLE );

        if( err != RFAL_ERR_NONE )
        {
            return false;
        }

        state = DEMO_ST_START_DISCOVERY;
        return true;
    }
    return false;
}

/*!
 *****************************************************************************
 * \brief Demo Cycle
 *
 *  This function executes the demo state machine.
 *  It must be called periodically
 *****************************************************************************
 */
void demoCycle( void )
{
    static rfalNfcDevice *nfcDevice;

    rfalNfcWorker();                                    /* Run RFAL worker periodically */

#if defined(PLATFORM_USER_BUTTON_PORT) && defined(PLATFORM_USER_BUTTON_PIN)
    /*******************************************************************************/
    /* Check if USER button is pressed */
    if( platformGpioIsLow(PLATFORM_USER_BUTTON_PORT, PLATFORM_USER_BUTTON_PIN))
    {
        discParam.wakeupEnabled = !discParam.wakeupEnabled;    /* enable/disable wakeup */
        state = DEMO_ST_START_DISCOVERY;                       /* restart loop          */
        platformLog("Toggling Wake Up mode %s\r\n", discParam.wakeupEnabled ? "ON": "OFF");

        /* Debounce button */
        while( platformGpioIsLow(PLATFORM_USER_BUTTON_PORT, PLATFORM_USER_BUTTON_PIN) );
    }
#endif /* PLATFORM_USER_BUTTON_PIN */

    switch( state )
    {
        /*******************************************************************************/
        case DEMO_ST_START_DISCOVERY:

          platformLedOff(PLATFORM_LED_A_PORT, PLATFORM_LED_A_PIN);
          platformLedOff(PLATFORM_LED_B_PORT, PLATFORM_LED_B_PIN);
          platformLedOff(PLATFORM_LED_F_PORT, PLATFORM_LED_F_PIN);
          platformLedOff(PLATFORM_LED_V_PORT, PLATFORM_LED_V_PIN);
          platformLedOff(PLATFORM_LED_AP2P_PORT, PLATFORM_LED_AP2P_PIN);
          platformLedOff(PLATFORM_LED_FIELD_PORT, PLATFORM_LED_FIELD_PIN);

          rfalNfcDeactivate( RFAL_NFC_DEACTIVATE_IDLE );
          rfalNfcDiscover( &discParam );

          multiSel = false;
          state    = DEMO_ST_DISCOVERY;
          break;

        /*******************************************************************************/
        case DEMO_ST_DISCOVERY:

            if( rfalNfcIsDevActivated( rfalNfcGetState() ) )
            {
                rfalNfcGetActiveDevice( &nfcDevice );

                switch( nfcDevice->type )
                {
                    /*******************************************************************************/
                    case RFAL_NFC_LISTEN_TYPE_NFCA:

                        platformLedOn(PLATFORM_LED_A_PORT, PLATFORM_LED_A_PIN);
                        switch( nfcDevice->dev.nfca.type )
                        {
                            case RFAL_NFCA_T1T:
                                platformLog("ISO14443A/Topaz (NFC-A T1T) TAG found. UID: %s\r\n", hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                                break;

                            case RFAL_NFCA_T4T:
                                platformLog("NFCA Passive ISO-DEP device found. UID: %s\r\n", hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );

                                demoAPDU();
                                break;

                            case RFAL_NFCA_T4T_NFCDEP:
                            case RFAL_NFCA_NFCDEP:
                                platformLog("NFCA Passive P2P device found. NFCID: %s\r\n", hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );

                                demoP2P( nfcDevice );
                                break;

                            default:
                                platformLog("ISO14443A/NFC-A card found. UID: %s\r\n", hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );

                                demoT2t();
                                break;
                        }
                        break;

                    /*******************************************************************************/
                    case RFAL_NFC_LISTEN_TYPE_NFCB:

                        platformLog("ISO14443B/NFC-B card found. UID: %s\r\n", hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                        platformLedOn(PLATFORM_LED_B_PORT, PLATFORM_LED_B_PIN);

                        if( rfalNfcbIsIsoDepSupported( &nfcDevice->dev.nfcb ) )
                        {
                            demoAPDU();
                        }
                        break;

                    /*******************************************************************************/
                    case RFAL_NFC_LISTEN_TYPE_NFCF:

                        if( rfalNfcfIsNfcDepSupported( &nfcDevice->dev.nfcf ) )
                        {
                            platformLog("NFCF Passive P2P device found. NFCID: %s\r\n", hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ) );
                            demoP2P( nfcDevice );
                        }
                        else
                        {
                            platformLog("Felica/NFC-F card found. UID: %s\r\n", hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ));

                            demoNfcf( &nfcDevice->dev.nfcf );
                        }

                        platformLedOn(PLATFORM_LED_F_PORT, PLATFORM_LED_F_PIN);
                        break;

                    /*******************************************************************************/
                    case RFAL_NFC_LISTEN_TYPE_NFCV:
                        {
                            uint8_t devUID[RFAL_NFCV_UID_LEN];

                            ST_MEMCPY( devUID, nfcDevice->nfcid, nfcDevice->nfcidLen );   /* Copy the UID into local var */
                            REVERSE_BYTES( devUID, RFAL_NFCV_UID_LEN );                 /* Reverse the UID for display purposes */
                          // platformLog("ISO15693/NFC-V card found. UID: %s\r\n", hex2Str(devUID, RFAL_NFCV_UID_LEN));

                            platformLedOn(PLATFORM_LED_V_PORT, PLATFORM_LED_V_PIN);

                            demoNfcv( &nfcDevice->dev.nfcv );
                        }
                        break;

                    /*******************************************************************************/
                    case RFAL_NFC_LISTEN_TYPE_ST25TB:

                        platformLog("ST25TB card found. UID: %s\r\n", hex2Str( nfcDevice->nfcid, nfcDevice->nfcidLen ));
                        platformLedOn(PLATFORM_LED_B_PORT, PLATFORM_LED_B_PIN);
                        break;

                    /*******************************************************************************/
                    case RFAL_NFC_LISTEN_TYPE_AP2P:
                    case RFAL_NFC_POLL_TYPE_AP2P:

                        platformLog("NFC Active P2P device found. NFCID3: %s\r\n", hex2Str(nfcDevice->nfcid, nfcDevice->nfcidLen));
                        platformLedOn(PLATFORM_LED_AP2P_PORT, PLATFORM_LED_AP2P_PIN);

                        demoP2P( nfcDevice );
                        break;

                    /*******************************************************************************/
                    case RFAL_NFC_POLL_TYPE_NFCA:
                    case RFAL_NFC_POLL_TYPE_NFCF:

                        platformLog("Activated in CE %s mode.\r\n", (nfcDevice->type == RFAL_NFC_POLL_TYPE_NFCA) ? "NFC-A" : "NFC-F");
                        platformLedOn( ((nfcDevice->type == RFAL_NFC_POLL_TYPE_NFCA) ? PLATFORM_LED_A_PORT : PLATFORM_LED_F_PORT),
                                       ((nfcDevice->type == RFAL_NFC_POLL_TYPE_NFCA) ? PLATFORM_LED_A_PIN  : PLATFORM_LED_F_PIN)  );

                        demoCE( nfcDevice );
                        break;

                    /*******************************************************************************/
                    default:
                        break;
                }

                rfalNfcDeactivate( RFAL_NFC_DEACTIVATE_IDLE );

#if !defined(DEMO_NO_DELAY_IN_DEMOCYCLE)
                switch( nfcDevice->type )
                {
                    case RFAL_NFC_POLL_TYPE_NFCA:
                    case RFAL_NFC_POLL_TYPE_NFCF:
                        break; /* When being in card emulation don't delay as some polling devices (phones) rely on tags to be re-discoverable */
                    default:
                        platformDelay(500); /* Delay before re-starting polling loop to not flood the UART log with re-discovered tags */
                }
#endif /* DEMO_NO_DELAY_IN_DEMOCYCLE */

                state = DEMO_ST_START_DISCOVERY;
            }
            break;

        /*******************************************************************************/
        case DEMO_ST_NOTINIT:
        default:
            break;
    }
}

static void demoCE( rfalNfcDevice *nfcDev )
{
#if RFAL_SUPPORT_CE && RFAL_FEATURE_LISTEN_MODE

    ReturnCode err = RFAL_ERR_NONE;
    uint8_t *rxData;
    uint16_t *rcvLen;
    uint8_t  txBuf[150];
    uint16_t txLen;

    do
    {
        rfalNfcWorker();

        switch( rfalNfcGetState() )
        {
            case RFAL_NFC_STATE_ACTIVATED:
                err = demoTransceiveBlocking( NULL, 0, &rxData, &rcvLen, 0);
                break;

            case RFAL_NFC_STATE_DATAEXCHANGE:
            case RFAL_NFC_STATE_DATAEXCHANGE_DONE:

                txLen = ( (nfcDev->type == RFAL_NFC_POLL_TYPE_NFCA) ? demoCeT4T( rxData, *rcvLen, txBuf, sizeof(txBuf) ) : rfalConvBytesToBits( demoCeT3T( rxData, rfalConvBitsToBytes(*rcvLen), txBuf, sizeof(txBuf) ) ) );
                err   = demoTransceiveBlocking( txBuf, txLen, &rxData, &rcvLen, RFAL_FWT_NONE );
                break;

            case RFAL_NFC_STATE_START_DISCOVERY:
                return;

            case RFAL_NFC_STATE_LISTEN_SLEEP:
            default:
                break;
        }
    }
    while( (err == RFAL_ERR_NONE) || (err == RFAL_ERR_SLEEP_REQ) );

#else
    NO_WARNING(nfcDev);
#endif /* RFAL_SUPPORT_CE && RFAL_FEATURE_LISTEN_MODE */
}

/*!
 *****************************************************************************
 * \brief Demo NFC-F
 *
 * Example how to exchange read and write blocks on a NFC-F tag
 *
 *****************************************************************************
 */
static void demoNfcf( rfalNfcfListenDevice *nfcfDev )
{
#if RFAL_FEATURE_NFCF

    ReturnCode                 err;
    uint8_t                    buf[ (RFAL_NFCF_NFCID2_LEN + RFAL_NFCF_CMD_LEN + (3*RFAL_NFCF_BLOCK_LEN)) ];
    uint16_t                   rcvLen;
    rfalNfcfServ               srv = RFAL_NFCF_SERVICECODE_RDWR;
    rfalNfcfBlockListElem      bl[3];
    rfalNfcfServBlockListParam servBlock;
    //uint8_t                    wrData[] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };

    servBlock.numServ   = 1;                            /* Only one Service to be used           */
    servBlock.servList  = &srv;                         /* Service Code: NDEF is Read/Writeable  */
    servBlock.numBlock  = 1;                            /* Only one block to be used             */
    servBlock.blockList = bl;
    bl[0].conf     = RFAL_NFCF_BLOCKLISTELEM_LEN_BIT;   /* Two-byte Block List Element           */
    bl[0].blockNum = 0x0001;                            /* Block: NDEF Data                      */

    err = rfalNfcfPollerCheck( nfcfDev->sensfRes.NFCID2, &servBlock, buf, sizeof(buf), &rcvLen);
    platformLog(" Check Block: %s Data:  %s \r\n", (err != RFAL_ERR_NONE) ? "FAIL": "OK", (err != RFAL_ERR_NONE) ? "" : hex2Str( &buf[1], RFAL_NFCF_BLOCK_LEN) );

    #if 0  /* Writing example */
        err = rfalNfcfPollerUpdate( nfcfDev->sensfRes.NFCID2, &servBlock, buf , sizeof(buf), wrData, buf, sizeof(buf) );
        platformLog(" Update Block: %s Data: %s \r\n", (err != RFAL_ERR_NONE) ? "FAIL": "OK", (err != RFAL_ERR_NONE) ? "" : hex2Str( wrData, RFAL_NFCF_BLOCK_LEN) );
        err = rfalNfcfPollerCheck( nfcfDev->sensfRes.NFCID2, &servBlock, buf, sizeof(buf), &rcvLen);
        platformLog(" Check Block:  %s Data: %s \r\n", (err != RFAL_ERR_NONE) ? "FAIL": "OK", (err != RFAL_ERR_NONE) ? "" : hex2Str( &buf[1], RFAL_NFCF_BLOCK_LEN) );
    #endif

#endif /* RFAL_FEATURE_NFCF */
}

/*!
 *****************************************************************************
 * \brief Demo NFC-V Exchange
 *
 * Example how to exchange read and write blocks on a NFC-V tag
 *
 *****************************************************************************
 */
static void demoNfcv( rfalNfcvListenDevice *nfcvDev )
{
#if RFAL_FEATURE_NFCV

    ReturnCode err;
    uint16_t rcvLen;
    uint8_t configBlock = 0x37;
    uint8_t	gpioBlock = 0x39;
    uint8_t	statusBlock = 0xA0;
    uint8_t rxBuf[ 1 + DEMO_NFCV_BLOCK_LEN + RFAL_CRC_LEN ];                        /* Flags + Block Data + CRC */
    uint8_t *uid;
    uint8_t reqFlag;
    uint8_t wrConfigData[1 + DEMO_NFCV_BLOCK_LEN] = {0x37, 0x00, 0x20, 0x1F, 0x00 };            /* CONFIG addr + 4 bytes settings */
    uint8_t	wrGPIOData[1 + DEMO_NFCV_BLOCK_LEN] = {0xA3, 0x14, 0x00, 0x00, 0x00 };				/* PWM_GPIO_CONFIG addr + 4 bytes settings */
    uint8_t wrEHConfig[1 + DEMO_NFCV_BLOCK_LEN] = {0x3d, 0x09, 0x00, 0x00, 0x00 };				/* Energy harvesting addr + 4 bytes settings */
    static bool gpioLowFlag = false;



    uid     = nfcvDev->InvRes.UID;
    reqFlag = RFAL_NFCV_REQ_FLAG_DEFAULT;

    /* Write settings to CONFIG, PWM_GPIO0_CONFIG, and EH_CONFIG */
    err = rfalNfcvPollerTransceiveReq(0xc1, 0x02, 0x04, NULL, wrConfigData, sizeof(wrConfigData), rxBuf, sizeof(rxBuf), &rcvLen);
    //platformLog(" Write CONFIG Block:\t\t%s\tData: %s\r\n", (err != RFAL_ERR_NONE) ? "FAIL": "OK", hex2Str( wrConfigData, DEMO_NFCV_BLOCK_LEN+1) );

    err = rfalNfcvPollerTransceiveReq(0xc1, 0x02, 0x04, NULL, wrGPIOData, sizeof(wrGPIOData), rxBuf, sizeof(rxBuf), &rcvLen);
    //platformLog(" Write PWM_GPIO_CONFIG Block:\t%s\tData: %s\r\n", (err != RFAL_ERR_NONE) ? "FAIL": "OK", hex2Str( wrGPIOData, DEMO_NFCV_BLOCK_LEN + 1) );

    err = rfalNfcvPollerTransceiveReq(0xc1, 0x02, 0x04, NULL, wrEHConfig, sizeof(wrEHConfig), rxBuf, sizeof(rxBuf), &rcvLen);
    //platformLog(" Write EH_CONFIG Block:\t\t%s\tData: %s\r\n", (err != RFAL_ERR_NONE) ? "FAIL": "OK", hex2Str( wrEHConfig, DEMO_NFCV_BLOCK_LEN + 1) );

    /* Read back all the configuration registers, and also the STATUS register
     * STATUS = STATUS0_REG, STATUS1_REG, RFU, RFU
     * 		In STATUS1_REG:
     * 						bit 7	VCC_BOOT_OK
     * 						bit 6	NFC_BOOT_OK
     * 						bit 5	RFU
     * 						bit 4	GPIO1_IN_STATUS
     * 						bit 3	GPIO0_IN_STATUS <-- with standard pull up config, 0b means button pushed
     * 						bit 2, 1, 0 don't matter
     */
	uint8_t txBuff[2] = {0x37, 0x00};
	err = rfalNfcvPollerTransceiveReq(0xc0, 0x02, 0x04, NULL, txBuff, sizeof(txBuff), rxBuf, sizeof(rxBuf), &rcvLen);
	//err = rfalNfcvPollerReadSingleBlock(reqFlag, uid, configBlock, rxBuf, sizeof(rxBuf), &rcvLen);
   // platformLog(" Read CONFIG Block:\t\t%s %s\r\n", (err != RFAL_ERR_NONE) ? "FAIL": "OK\tData:", (err != RFAL_ERR_NONE) ? "" : hex2Str( &rxBuf[1], DEMO_NFCV_BLOCK_LEN));

    memcpy(txBuff, (uint8_t[]){0x39, 0x00}, sizeof(txBuff));
    err = rfalNfcvPollerTransceiveReq(0xc0, 0x02, 0x04, NULL, txBuff, sizeof(txBuff), rxBuf, sizeof(rxBuf), &rcvLen);
    //err = rfalNfcvPollerReadSingleBlock(reqFlag, uid, gpioBlock, rxBuf, sizeof(rxBuf), &rcvLen);
    //platformLog(" Read PWM_GPIO_CONFIG Block:\t%s %s\r\n", (err != RFAL_ERR_NONE) ? "FAIL": "OK\tData:", (err != RFAL_ERR_NONE) ? "" : hex2Str( &rxBuf[1], DEMO_NFCV_BLOCK_LEN));

    memcpy(txBuff, (uint8_t[]){0x3d, 0x00}, sizeof(txBuff));
	err = rfalNfcvPollerTransceiveReq(0xc0, 0x02, 0x04, NULL, txBuff, sizeof(txBuff), rxBuf, sizeof(rxBuf), &rcvLen);
	//err = rfalNfcvPollerReadSingleBlock(reqFlag, uid, gpioBlock, rxBuf, sizeof(rxBuf), &rcvLen);
	//platformLog(" Read EH_CONFIG Block:\t\t%s %s\r\n", (err != RFAL_ERR_NONE) ? "FAIL": "OK\tData:", (err != RFAL_ERR_NONE) ? "" : hex2Str( &rxBuf[1], DEMO_NFCV_BLOCK_LEN));

    memcpy(txBuff, (uint8_t[]){0xA0, 0x00}, sizeof(txBuff));
    err = rfalNfcvPollerTransceiveReq(0xc0, 0x02, 0x04, NULL, txBuff, sizeof(txBuff), rxBuf, sizeof(rxBuf), &rcvLen);
	//platformLog(" Read STATUS Block:\t%s \r\n", (err != RFAL_ERR_NONE) ? "FAIL": "OK\t");
  //  platformLog("KEY \r\n");

    if(rxBuf[2] == 0xC0 && !gpioLowFlag)
	{
		platformLog("KEY ON\r\n");
		gpioLowFlag = true;
		discParam.techs2Find |= RFAL_NFC_POLL_TECH_AP2P;

        discParam.techs2Find |= RFAL_NFC_LISTEN_TECH_AP2P;
       discParam.techs2Find &= ~(RFAL_NFC_POLL_TECH_V);

		HAL_GPIO_WritePin(KEY_LED_GPIO_Port, KEY_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RTN_LED_GPIO_Port, RTN_LED_Pin, GPIO_PIN_SET);
		// TODO: RTN signal goes low
	}
	else if(rxBuf[2] == 0xC8 && gpioLowFlag)
	{
		//gpioLowFlag = false;
		platformLog("KEY OFF\r\n");
		//discParam.techs2Find &= (RFAL_NFC_POLL_TECH_V);
		HAL_GPIO_WritePin(KEY_LED_GPIO_Port, KEY_LED_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RTN_LED_GPIO_Port, RTN_LED_Pin, GPIO_PIN_RESET);
	}

#endif /* RFAL_FEATURE_NFCV */
}

/*!
 *****************************************************************************
 * \brief Demo T2T Exchange
 *
 * Example how to exchange read blocks on a T2T tag
 *
 *****************************************************************************
 */
static void demoT2t( void )
{
#if RFAL_FEATURE_T2T
    ReturnCode            err;
    uint16_t              rcvLen;
    uint8_t               blockNum = 0;
    uint8_t               rxBuf[ RFAL_T2T_READ_DATA_LEN ];

    err = rfalT2TPollerRead(blockNum, rxBuf, sizeof(rxBuf), &rcvLen);
    platformLog(" Read Block: %s %s\r\n", (err != RFAL_ERR_NONE) ? "FAIL": "OK Data:", (err != RFAL_ERR_NONE) ? "" : hex2Str( rxBuf, RFAL_T2T_READ_DATA_LEN));

#endif
}

/*!
 *****************************************************************************
 * \brief Demo P2P Exchange
 *
 * Sends a NDEF URI record 'http://www.ST.com' via NFC-DEP (P2P) protocol.
 *
 * This method sends a set of static predefined frames which tries to establish
 * a LLCP connection, followed by the NDEF record, and then keeps sending
 * LLCP SYMM packets to maintain the connection.
 *
 *
 *****************************************************************************
 */
void demoP2P( rfalNfcDevice *nfcDev )
{
#if RFAL_FEATURE_NFC_DEP

    uint16_t   *rxLen;
    uint8_t    *rxData;
    ReturnCode err;

    /* In Listen mode retrieve the first request from Initiator */
    if( nfcDev->type == RFAL_NFC_POLL_TYPE_AP2P )
    {
        demoTransceiveBlocking( NULL, 0, &rxData, &rxLen, 0);

        /* Initiator request is being ignored/discarded  */
    }

    platformLog(" Initialize device .. ");
    err = demoTransceiveBlocking( ndefInit, sizeof(ndefInit), &rxData, &rxLen, RFAL_FWT_NONE);
    if( err != RFAL_ERR_NONE )
    {
        platformLog("failed.\r\n");
        return;
    }
    platformLog("succeeded.\r\n");

    platformLog(" Device present, maintaining connection ");

	NFC_CAN_DataExchange();

#endif /* RFAL_FEATURE_NFC_DEP */
}

/*!
 *****************************************************************************
 * \brief Demo APDUs Exchange
 *
 * Example how to exchange a set of predefined APDUs with PICC. The NDEF
 * application will be selected and then CC will be selected and read.
 *
 *****************************************************************************
 */
void demoAPDU( void )
{
#if RFAL_FEATURE_ISO_DEP_POLL
    ReturnCode err;
    uint16_t   *rxLen;
    uint8_t    *rxData;

    /* Exchange APDU: NDEF Tag Application Select command */
    err = demoTransceiveBlocking( ndefSelectApp, sizeof(ndefSelectApp), &rxData, &rxLen, RFAL_FWT_NONE );
    platformLog(" Select NDEF Application: %s Data: %s\r\n", (err != RFAL_ERR_NONE) ? "FAIL": "OK", (err != RFAL_ERR_NONE) ? "" : hex2Str( rxData, *rxLen) );

    if( (err == RFAL_ERR_NONE) && rxData[0] == 0x90 && rxData[1] == 0x00)
    {
        /* Exchange APDU: Select Capability Container File */
        err = demoTransceiveBlocking( ccSelectFile, sizeof(ccSelectFile), &rxData, &rxLen, RFAL_FWT_NONE );
        platformLog(" Select CC: %s Data: %s\r\n", (err != RFAL_ERR_NONE) ? "FAIL": "OK", (err != RFAL_ERR_NONE) ? "" : hex2Str( rxData, *rxLen) );

        /* Exchange APDU: Read Capability Container File  */
        err = demoTransceiveBlocking( readBinary, sizeof(readBinary), &rxData, &rxLen, RFAL_FWT_NONE );
        platformLog(" Read CC: %s Data: %s\r\n", (err != RFAL_ERR_NONE) ? "FAIL": "OK", (err != RFAL_ERR_NONE) ? "" : hex2Str( rxData, *rxLen) );
    }
#endif /* RFAL_FEATURE_ISO_DEP_POLL */
}




/**
  * @}
  */




/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

