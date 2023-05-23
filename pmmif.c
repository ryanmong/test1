/****************************************************************************

    @file pmmif.c

    @par PMMIF

    @brief This module implements PMMIF interface

    @par Module Design Description

    @par Revision History
        See revision history in configuration management repository.

    @par Copyright
        Copyright (c) 2012 Invensys Rail Group.
        Unpublished Work, All Rights Reserved.
        This work is protected by copyright and the information contained
        herein is confidential. The work may not be copied and the information
        herein may not be disclosed except by the written permission of, and
        in a manner permitted by the Invensys Rail Group.

 *****************************************************************************/

/*****************************************************************************
* INCLUDE FILES
*****************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <pthread.h>

#include "fs.h"
#include "pmmif.h"
#include "iniparser.h"
#include "pmmidx.h"
#include "pmm_drvr_api.h"
#include "appMain.h"
#include "priority.h"
#include "threadmon.h"
#include "crc32.h"
#include "util.h"
#include "paramCrypto.h"

#define PMM_BYTES_PER_PAGE   4320

/*****************************************************************************
* TYPE DEFINITIONS
*****************************************************************************/
typedef enum
{
    PMMIF_READLOG_STATE_IDLE,
    PMMIF_READLOG_STATE_READ_RANGE_REQUESTED,
    PMMIF_READLOG_STATE_READ_PCT_REQUESTED,
    PMMIF_READLOG_STATE_READ_NEXT,
    PMMIF_READLOG_STATE_WAIT_CONTINUE
} PMMIF_READLOG_STATE_T;

typedef enum
{
    PMMIF_READTYPE_NONE,
    PMMIF_READTYPE_RANGE,
    PMMIF_READTYPE_PCT
} PMMIF_READTYPE_T;

typedef struct PMMIF_TASK_CFG_TAG
{
    const char            *threadName;
    int                   priority;
    PMMIF_PARTITION_SEL_T partId;
} PMMIF_TASK_CFG_T;

/*****************************************************************************
* CONSTANTS
*****************************************************************************/
enum PMMIF_INI_FLD_NUMS
{
    PMMIF_INIFLD_MFG_INSTALLED_MEM,
    PMMIF_INIFLD_MFG_ENABLED_MEM
};

static const PMMIF_TASK_CFG_T mThreadCfg[] =
{
    { "pmmif_dpart1", PRI_MEDIUM, PMMIF_EVENT_DATA_PARTITION_1 },

    /*
        Need to figure out how to do this configuration unique
        to each application. First thought is to ifdef it


        { "pmmif_dpart2", PRI_MEDIUM, PMMIF_EVENT_DATA_PARTITION_2 },
        { "pmmif_vpart1", PRI_MEDIUM, PMMIF_VIDEO_DATA_PARTITION_1 }
     */
};

static const char *gINIKeyTable[] =   /* Data dictionary for .ini file */
{
    "mfg data:data 1",  /* PMMIF_INIFLD_MFG_INSTALLED_MEM */
    "mfg data:data 2"   /* PMMIF_INIFLD_MFG_ENABLED_MEM */
};

/*****************************************************************************
* VARIABLES
*****************************************************************************/
static bool                         mThreadStarupComplete[PMMIF_MAX_NUM_PARTITIONS];
static PMMIF_READLOG_STATE_T        mReadLogState[PMMIF_MAX_NUM_PARTITIONS];
static uint8                        mReadLogPct[PMMIF_MAX_NUM_PARTITIONS];
static ENTRY_READ_CALLBACK_FUNC_PTR mReadCallBackFuncPtr[PMMIF_MAX_NUM_PARTITIONS];
static void                         *mCallbackArg[PMMIF_MAX_NUM_PARTITIONS];
static PMMIF_READ_RANGE_T           mReadRangeData[PMMIF_MAX_NUM_PARTITIONS];
static uint8                        mReadType[PMMIF_MAX_NUM_PARTITIONS];
static uint32                       mHwInstalledMemGiB;
static uint32                       mHwEnabledMemGiB;

/*****************************************************************************
* PRIVATE PROTOTYPES
*****************************************************************************/
static void *pmmif_ReadEventsTask(void *iParam);
static PMMIF_ENTRY_HDR_T *pmmif_ReadEntry(PMMIF_PARTITION_SEL_T iPartId);
static bool pmmif_ReadEntryAndGetPMMIFTimestamp(PMMIF_PARTITION_SEL_T iPartId,
        PMMIF_TIMESTAMP_T *oTimeStampPtr);
static bool pmmif_FindReadPctStart(PMMIF_PARTITION_SEL_T iPartId);
static bool pmmif_FindReadRangeStart(int32 iThreadId,PMMIF_PARTITION_SEL_T iPartId, PMMIF_ENTRY_HDR_T **oEntryPtr);
static bool pmmif_ValidateRangeTimes(PMMIF_PARTITION_SEL_T iPartId);

static void pmmif_LoadPMMInfo(void);
static bool pmmif_GetEncryptedParam(uint32 iSerialNo, const char *iEncryptedText, uint8 iValueId, uint32 *iRetValue);

int pmmif_CmpTimestamp(const PMMIF_TIMESTAMP_T *iTimestamp1Ptr, const PMMIF_TIMESTAMP_T *iTimestamp2Ptr);

/*****************************************************************************
* FUNCTIONS
*****************************************************************************/

#if 0
static void print_timestamp(PMMIF_TIMESTAMP_T t, bool cr)
{
	char buffer[48];

	sprintf(buffer, "%02X%02X%02X%02X%02X%02X.%02X%02X",
			t.bcdDateTime.yearBCD,
			t.bcdDateTime.monthBCD,
			t.bcdDateTime.dayBCD,
			t.bcdDateTime.hourBCD,
			t.bcdDateTime.minBCD,
			t.bcdDateTime.secBCD,
			t.bcdDateTime.hundSecBCD,
			t.milliSec
			);

	printf("%s", buffer);
	if( cr ) printf("\n");
}
#endif

/******************************************************************************
    @brief      This function takes a PMM address and converts it to an index

    @param      address - the PMM Address

    @return     an index

    NOTE:       An index makes the fpga address space linear. An Index has the
                following structure:

                Bits		Description
                -----------------------
                 6 - 0		page (0 - 127)
                16 - 7      block (0 - 1023)
                17          plane (0-1)
                24 - 18     chip  (1-16)

******************************************************************************/

uint32 address_to_index(uint32 address)
{
	uint32 result = 0;

	result = (((address >> 24) & 0x07)   << 18) + // Chip
	         (((address >> 7)  & 0x01)   << 17) + // Plane
	         (((address >> 8)  & 0x03FF) <<  7) + // Block
	         (((address >> 0)  & 0x7F)   <<  0);  // Page

	return result;
}

/******************************************************************************
    @brief      This function takes an index and calculates a PMM address

    @param      index - the page as an index

    @return     the PMM Adress of that index


******************************************************************************/

uint32 index_to_address(uint32 index)
{
	uint32 result = 0;

	uint32 the_page;
	uint32 the_plane;
	uint32 the_block;
	uint32 the_chip;

	the_chip  = (index >> 18) & 0x07;
	the_plane = (index >> 17) & 0x01;
	the_block = (index >>  7) & 0x03FF;
	the_page  = (index >>  0) & 0x7F;

	result = (the_chip << 24) | (the_block << 8) | (the_plane << 7) | (the_page);

	return result;
}

/******************************************************************************
    @brief      This function initializes the PMMIF module.

    @param      iArgc - command line argument count
    @param      iArgv - pointer to command line arguments

    @return     TRUE if successful, otherwise FALSE

******************************************************************************/
bool PMMIF_Init(int iArgc, char *iArgv[])
{
    uint32             lIndex;
    pthread_attr_t     lThreadAttributes;
    struct sched_param lSchedulerParams;
    pthread_t          lThreadHndl;

    unused(iArgc);
    unused(iArgv);

    pmmif_LoadPMMInfo();

    fprintf(stderr, "PMMIF_Init() 1:%ld 2:%ld\r\n", mHwInstalledMemGiB, mHwEnabledMemGiB);

    if (TRUE != PMM_DRV_API_Init(mHwInstalledMemGiB, mHwEnabledMemGiB))
    {
        return FALSE;
    }

    for (lIndex = 0; lIndex < PMMIF_MAX_NUM_PARTITIONS; lIndex++)
    {
        mThreadStarupComplete[lIndex] = FALSE;
        mReadLogState[lIndex]         = PMMIF_READLOG_STATE_IDLE;

        /* create a thread to interface to handle read requests */
        pthread_attr_init(&lThreadAttributes);
        pthread_attr_setdetachstate(&lThreadAttributes, PTHREAD_CREATE_DETACHED);

        assert(0 == pthread_create(&lThreadHndl,
                                   &lThreadAttributes,
                                   pmmif_ReadEventsTask,
                                   (void *) &mThreadCfg[lIndex]));

        /* Set the scheduling policy to the real-time round robin option. */
        lSchedulerParams.sched_priority = mThreadCfg[lIndex].priority;
        assert(0 == pthread_setschedparam(lThreadHndl, SCHED_RR, &lSchedulerParams));

        /* wait until the thread created is finished with its startup */
        while (TRUE != mThreadStarupComplete[lIndex])
        {
            usleep(2000);
        }
    }

    return TRUE;
}


/******************************************************************************
    @brief

    @return     TRUE if PMM is currently writable, otherwise FALSE

******************************************************************************/
bool PMMIF_IsPMMInReadOnlyMode(PMMIF_PARTITION_SEL_T iPartId)
{
    assert(iPartId < PMMIF_MAX_NUM_PARTITIONS);

    return PMM_DRV_API_IsPMMInReadOnlyMode(iPartId);
}


/******************************************************************************
    @brief

    @return

******************************************************************************/
bool PMMIF_BeginEventReadRange(PMMIF_PARTITION_SEL_T iPartId,
                               const PMMIF_READ_RANGE_T *iReadRangePtr,
                               const ENTRY_READ_CALLBACK_FUNC_PTR iCallbackPtr,
                               void *iCallbackArgPtr)
{
    assert(NULL != iCallbackPtr);
    assert(iPartId < PMMIF_MAX_NUM_PARTITIONS);

    if (PMMIF_READLOG_STATE_IDLE != mReadLogState[iPartId])
    {
        return FALSE;
    }

    /* tell the driver that we want to read from the PMM. When we are
       reading, no more writing is going to take place
     */
    if (TRUE != PMM_DRV_API_SetPMMReadMode(iPartId, APP_GetMyPID()))
    {
        return FALSE;
    }

    APP_SetState(CHMM_STATE_OFFLINE, __FILE__, __LINE__);

    memcpy(&mReadRangeData[iPartId], iReadRangePtr, sizeof(mReadRangeData[iPartId]));
    mReadCallBackFuncPtr[iPartId] = iCallbackPtr;
    mCallbackArg[iPartId]         = iCallbackArgPtr;

    mReadLogState[iPartId] = PMMIF_READLOG_STATE_READ_RANGE_REQUESTED;

    return TRUE;
}


/******************************************************************************
    @brief

    @return

******************************************************************************/
bool PMMIF_BeginEventReadPct(PMMIF_PARTITION_SEL_T iPartId,
                             uint8 iReadPct,
                             const ENTRY_READ_CALLBACK_FUNC_PTR iCallbackPtr,
                             void *iCallbackArgPtr)
{
    assert((iReadPct > 0) && (iReadPct <= 100));
    assert(NULL != iCallbackPtr);
    assert(iPartId < PMMIF_MAX_NUM_PARTITIONS);

    if (PMMIF_READLOG_STATE_IDLE != mReadLogState[iPartId])
    {
        return FALSE;
    }

    /* tell the driver that we want to read from the PMM. When we are
       reading, no more writing is going to take place
     */
    if (TRUE != PMM_DRV_API_SetPMMReadMode(iPartId, APP_GetMyPID()))
    {
        return FALSE;
    }

    APP_SetState(CHMM_STATE_OFFLINE, __FILE__, __LINE__);

    mReadCallBackFuncPtr[iPartId] = iCallbackPtr;
    mReadLogPct[iPartId]          = iReadPct;
    mCallbackArg[iPartId]         = iCallbackArgPtr;

    mReadLogState[iPartId] = PMMIF_READLOG_STATE_READ_PCT_REQUESTED;

    return TRUE;
}


/******************************************************************************
    @brief

    @return

******************************************************************************/
bool PMMIF_EndEventRead(PMMIF_PARTITION_SEL_T iPartId)
{
    assert(iPartId < PMMIF_MAX_NUM_PARTITIONS);

    mReadLogState[iPartId] = PMMIF_READLOG_STATE_IDLE;

    if (TRUE != PMM_DRV_API_ClearPMMReadMode(iPartId, APP_GetMyPID()))
    {
        APP_SetState(CHMM_STATE_FAILED, __FILE__, __LINE__);
        return FALSE;
    }

    APP_SetState(CHMM_STATE_OPERATIONAL, __FILE__, __LINE__);

    return TRUE;
}


/******************************************************************************
    @brief

    @return

******************************************************************************/
bool PMMIF_ContinueEventRead(PMMIF_PARTITION_SEL_T iPartId)
{
    assert(iPartId < PMMIF_MAX_NUM_PARTITIONS);

    if (PMMIF_READLOG_STATE_WAIT_CONTINUE == mReadLogState[iPartId])
    {
        mReadLogState[iPartId] = PMMIF_READLOG_STATE_READ_NEXT;
        return TRUE;
    }

    return FALSE;
}


/******************************************************************************
    @brief      This function writes entry to PPM. This is a fake version
                that writes data to a filesystem file instead of the PMM

    @return     TRUE if successful, otherwise FALSE

******************************************************************************/
bool PMMIF_WriteEntry(PMMIF_PARTITION_SEL_T iPartId,
                      const void *iDataPtr,
                      const PMMIF_TIMESTAMP_T *iEntryTimestampPtr,
                      PMM_MEM_SIZE_T iDataLen)
{
    PMMIF_ENTRY_HDR_T lPMMHdr;

    assert(iPartId < PMMIF_MAX_NUM_PARTITIONS);

    /* see if someone has the file open for reading,
       suspend writing if that is the case */
    if (TRUE == PMM_DRV_API_IsPMMInReadOnlyMode(iPartId))
    {
        UTIL_LogStr("PMMIF_WriteEntry:Called while in incorrect state");
        return FALSE;
    }

    /* PMM is not in read only mode, ready for writing at this point */

    memset(&lPMMHdr, 0, sizeof(lPMMHdr));
    lPMMHdr.headerMagic = PMMIF_ENTRY_HEADER_MAGIC_NUM;
    lPMMHdr.entryLen    = iDataLen;
    memcpy(&lPMMHdr.timeStamp, iEntryTimestampPtr, sizeof(lPMMHdr.timeStamp));

    lPMMHdr.headerCRC = CRC_CalcCRC32(0xFFFFFFFF, &lPMMHdr, sizeof(lPMMHdr) - 4);

    if (TRUE != PMM_DRV_API_EventWrite(iPartId, &lPMMHdr, iDataPtr, iDataLen))
    {
        APP_SetState(CHMM_STATE_FAILED, __FILE__, __LINE__);
        return FALSE;
    }

    APP_SetState(CHMM_STATE_OPERATIONAL, __FILE__, __LINE__);
    return TRUE;
}


/******************************************************************************
    @brief

    @return

******************************************************************************/
static void *pmmif_ReadEventsTask(void *iParam)
{
    static char lTempBuf[200];

    PMMIF_ENTRY_HDR_T *lEntryHdrPtr = NULL;
    PMMIF_TASK_CFG_T  *lParamPtr;
    int32             lThreadMonId;


    assert(NULL != iParam);

    lParamPtr = (PMMIF_TASK_CFG_T *) iParam;
    assert(NULL != lParamPtr->threadName);

    assert(lParamPtr->partId < PMMIF_MAX_NUM_PARTITIONS);

    lThreadMonId = THREADMON_Register(lParamPtr->threadName, 5, lParamPtr->priority);


    mThreadStarupComplete[lParamPtr->partId] = TRUE;


    while (FALSE == APP_ShutdownCmdRecvd())
    {
        /* Keep the thread monitor happy. */
        THREADMON_Kick(lThreadMonId);

        /* if locked for reading, then update index.

           This is something that won't be done by this application in the
           real system, it will be done by a stand along program
         */

        switch (mReadLogState[lParamPtr->partId])
        {
            case PMMIF_READLOG_STATE_IDLE:
                usleep(2000);
                continue;
                break;

            case PMMIF_READLOG_STATE_READ_RANGE_REQUESTED:
                if (TRUE != pmmif_FindReadRangeStart(lThreadMonId,lParamPtr->partId, &lEntryHdrPtr))
                {
                    mReadType[lParamPtr->partId] = PMMIF_READTYPE_NONE;
                    mReadLogState[lParamPtr->partId] = PMMIF_READLOG_STATE_IDLE;
                    memset(&mReadRangeData[lParamPtr->partId], 0, sizeof(mReadRangeData[lParamPtr->partId]));
                    mReadCallBackFuncPtr[lParamPtr->partId](mCallbackArg[lParamPtr->partId], NULL);
                }
                else
                {
                	//send out the first event that was read from the result of the search
                    mReadType[lParamPtr->partId]     = PMMIF_READTYPE_RANGE;
                    mReadLogState[lParamPtr->partId] = PMMIF_READLOG_STATE_WAIT_CONTINUE;
                    mReadCallBackFuncPtr[lParamPtr->partId](mCallbackArg[lParamPtr->partId], lEntryHdrPtr);
                }

                break;

            case PMMIF_READLOG_STATE_READ_PCT_REQUESTED:
                if (TRUE != pmmif_FindReadPctStart(lParamPtr->partId))
                {
                    mReadCallBackFuncPtr[lParamPtr->partId](mCallbackArg[lParamPtr->partId], NULL);
                    mReadLogState[lParamPtr->partId] = PMMIF_READLOG_STATE_IDLE;
                    mReadLogPct[lParamPtr->partId]   = 0;
                    mReadType[lParamPtr->partId]     = PMMIF_READTYPE_NONE;
                }
                else
                {
                    mReadType[lParamPtr->partId]     = PMMIF_READTYPE_PCT;
                    mReadLogState[lParamPtr->partId] = PMMIF_READLOG_STATE_READ_NEXT;
                }

                break;

            case PMMIF_READLOG_STATE_READ_NEXT:

                /* the file should be positioned at the next event record header
                   read the header into a local */

                lEntryHdrPtr = pmmif_ReadEntry(lParamPtr->partId);
                if (NULL != lEntryHdrPtr)
                {
                    /* if the read request was a date range,
                       then need to determine if the entry just
                       retrieved is newer than the ending date.
                     */
                    if (PMMIF_READTYPE_RANGE == mReadType[lParamPtr->partId])
                    {
                        /* if the timestamp on the event entry just read
                           is newer than the end time then we are done */
                        if (pmmif_CmpTimestamp(&lEntryHdrPtr->timeStamp, &(mReadRangeData[lParamPtr->partId].endTime)) > 0)
                        {
                            free(lEntryHdrPtr);
                            lEntryHdrPtr = NULL;
                        }
                    }
                }

                if (NULL == lEntryHdrPtr)
                {
                    sprintf(lTempBuf, "pmmif_ReadEventsTask:Finished reading from file at line %d", __LINE__);
                    UTIL_LogStr(lTempBuf);
                    mReadCallBackFuncPtr[lParamPtr->partId](mCallbackArg[lParamPtr->partId], NULL);
                    mReadLogState[lParamPtr->partId] = PMMIF_READLOG_STATE_IDLE;
                }
                else
                {
                    mReadLogState[lParamPtr->partId] = PMMIF_READLOG_STATE_WAIT_CONTINUE;

                    /* call application so it can present this to user
                       note - the call back function is responsible for
                       freeing the memory !
                     */
                    mReadCallBackFuncPtr[lParamPtr->partId](mCallbackArg[lParamPtr->partId], lEntryHdrPtr);
                }
                break;

            case PMMIF_READLOG_STATE_WAIT_CONTINUE:
                /* don't do anything, timeouts etc. handled by application */
                break;

            default:
                break;
        }

        usleep(500);

    } /* end while as long as the task should run */

    PMM_DRV_API_Shutdown();

    THREADMON_Deregister(lThreadMonId);

    return NULL;
}


/******************************************************************************
    @brief      This function reads an event entry from the specifed partition
                at the current file position. The entry is stored in memory
                allocated by this function.

    @param      iPartId       - protected memory parition to operate on

    @return     pointer to event data read into allocated memory or NULL

******************************************************************************/
static PMMIF_ENTRY_HDR_T *pmmif_ReadEntry(PMMIF_PARTITION_SEL_T iPartId)
{
    static char       lTempBuf[200];
    PMMIF_ENTRY_HDR_T *lEntryHdrPtr;
    uint8             *lDataPtr = NULL;
    bool               done_reading = FALSE;

    assert(iPartId < PMMIF_MAX_NUM_PARTITIONS);

    lDataPtr = malloc(PMM_BYTES_PER_PAGE);
    if (NULL == lDataPtr)
    {
        sprintf(lTempBuf, "pmmif_ReadEntry: Unable to allocate memory at line %d", __LINE__);
        UTIL_LogStr(lTempBuf);

        return NULL;
    }

    /* Keep reading until error reading data or valid page is found or head pointer is reached. */
    /* (If a page is invalid, want to keep reading - don't stop on first bad page!) */
    while (!done_reading)
    {
        /* read it */
        /* If can't read page at all, get out immediately */
        if ( PMM_DRV_API_Read(iPartId, lDataPtr, PMM_BYTES_PER_PAGE) < sizeof(PMMIF_ENTRY_HDR_T))
        {
            sprintf(lTempBuf, "pmmif_ReadEntry: Unable to read page @ 0x%08lX offset 0x%08lX", (unsigned long) PMM_DRV_API_Tell(iPartId), PMM_DRV_API_GetHeadOffset(iPartId));
            UTIL_LogStr(lTempBuf);
            free(lDataPtr);
            return NULL;
        }

        lEntryHdrPtr = (PMMIF_ENTRY_HDR_T *)lDataPtr;

        /* Check header Magic# */
        if( lEntryHdrPtr->headerMagic != PMMIF_ENTRY_HEADER_MAGIC_NUM)
        {
            sprintf(lTempBuf, "pmmif_ReadEntry: Invalid page header (magic num) @ 0x%08lX", (unsigned long) PMM_DRV_API_Tell(iPartId));
            UTIL_LogStr(lTempBuf);
        }

        /* Check the  header CRC */
        else if (lEntryHdrPtr->headerCRC != CRC_CalcCRC32(0xFFFFFFFF, lEntryHdrPtr, sizeof(PMMIF_ENTRY_HDR_T) - 4))
        {
            sprintf(lTempBuf, "pmmif_ReadEntry: Invalid page header (CRC) @ 0x%08lX", (unsigned long) PMM_DRV_API_Tell(iPartId));
            UTIL_LogStr(lTempBuf);
        }
        else
        {
            done_reading = TRUE;
        }

        /* Now - if not done reading, had a problem.  See if head pointer has been reached yet. */
        /* If reached the head, done with no results.  Otherwise, read the next page. */
        if ((!done_reading) && (PMM_DRV_API_Tell(iPartId) == PMM_DRV_API_GetHeadOffset(iPartId)))
        {
            done_reading = TRUE;
            sprintf(lTempBuf, "pmmif_ReadEntry: Reached head pointer 0x%08lX", (unsigned long) PMM_DRV_API_Tell(iPartId));
            UTIL_LogStr(lTempBuf);
            free(lDataPtr);
            lDataPtr = NULL;
            done_reading = TRUE;
        }

    }

    return (PMMIF_ENTRY_HDR_T *) lDataPtr;
}


/******************************************************************************
    @brief      This function reads an event entry from the specifed partition
                at the current file position and fills the provided structure
                with a timestamp that is retrieved from the PMMIF header.

    @param      iPartId       - protected memory parition to operate on
    @param      oTimeStampPtr - pointer to structure to put application's
                                timestamp in

    @return     TRUE if entry can be read

******************************************************************************/
static bool pmmif_ReadEntryAndGetPMMIFTimestamp(PMMIF_PARTITION_SEL_T iPartId,
                                                PMMIF_TIMESTAMP_T *oTimeStampPtr)
{
    PMMIF_ENTRY_HDR_T *lTempEntryPtr;

    assert(iPartId < PMMIF_MAX_NUM_PARTITIONS);

    lTempEntryPtr = pmmif_ReadEntry(iPartId);
    if (NULL == lTempEntryPtr)
    {
        return FALSE;
    }

    memcpy(oTimeStampPtr, &lTempEntryPtr->timeStamp, sizeof(PMMIF_TIMESTAMP_T));
    free(lTempEntryPtr);

    return TRUE;
}


/******************************************************************************
    @brief      This function compares two timestamps and returns an integer
                result value like memcmp does.

                Value           Explanation
                -----           -----------
                less than 0     iTimestamp1Ptr is less than iTimestamp2Ptr
                equal to 0      iTimestamp1Ptr is equal to iTimestamp2Ptr
                greater than 0  iTimestamp1Ptr is greater than iTimestamp2Ptr

    @param      iTimestamp1Ptr - pointer to first timestamp
    @param      iTimestamp2Ptr - pointer to second timestamp

    @return     memcmp type return value

******************************************************************************/
int pmmif_CmpTimestamp(const PMMIF_TIMESTAMP_T *iTimestamp1Ptr, const PMMIF_TIMESTAMP_T *
                              iTimestamp2Ptr)
{
    /* if year's don't match */
    if (iTimestamp1Ptr->bcdDateTime.yearBCD != iTimestamp2Ptr->bcdDateTime.yearBCD)
    {
        /* if time 1 smaller than time 2 */
        if (FROM_BCD(iTimestamp1Ptr->bcdDateTime.yearBCD) < FROM_BCD(iTimestamp2Ptr->bcdDateTime.yearBCD))
        {
            return -1;
        }

        return 1;
    }

    /* if this point reached, then years match */
    if (iTimestamp1Ptr->bcdDateTime.monthBCD != iTimestamp2Ptr->bcdDateTime.monthBCD)
    {
        /* if time 1 smaller than time 2 */
        if (FROM_BCD(iTimestamp1Ptr->bcdDateTime.monthBCD) < FROM_BCD(iTimestamp2Ptr->bcdDateTime.monthBCD))
        {
            return -1;
        }

        return 1;
    }

    /* if this point reached, then years and month match */
    if (iTimestamp1Ptr->bcdDateTime.dayBCD != iTimestamp2Ptr->bcdDateTime.dayBCD)
    {
        /* if time 1 smaller than time 2 */
        if (FROM_BCD(iTimestamp1Ptr->bcdDateTime.dayBCD) < FROM_BCD(iTimestamp2Ptr->bcdDateTime.dayBCD))
        {
            return -1;
        }

        return 1;
    }

    /* if this point reached, then years, month and day match */
    if (iTimestamp1Ptr->bcdDateTime.hourBCD != iTimestamp2Ptr->bcdDateTime.hourBCD)
    {
        /* if time 1 smaller than time 2 */
        if (FROM_BCD(iTimestamp1Ptr->bcdDateTime.hourBCD) < FROM_BCD(iTimestamp2Ptr->bcdDateTime.hourBCD))
        {
            return -1;
        }

        return 1;
    }

    /* if this point reached, then years, month, days and hour match */
    if (iTimestamp1Ptr->bcdDateTime.minBCD != iTimestamp2Ptr->bcdDateTime.minBCD)
    {
        /* if time 1 smaller than time 2 */
        if (FROM_BCD(iTimestamp1Ptr->bcdDateTime.minBCD) < FROM_BCD(iTimestamp2Ptr->bcdDateTime.minBCD))
        {
            return -1;
        }

        return 1;
    }

    /* if this point reached, then years, month, days, hours, and min match */
    if (iTimestamp1Ptr->bcdDateTime.secBCD != iTimestamp2Ptr->bcdDateTime.secBCD)
    {
        /* if time 1 smaller than time 2 */
        if (FROM_BCD(iTimestamp1Ptr->bcdDateTime.secBCD) < FROM_BCD(iTimestamp2Ptr->bcdDateTime.secBCD))
        {
            return -1;
        }

        return 1;
    }

    /* if this point reached, then years, month, days, hours, min, and sec match */
    if (iTimestamp1Ptr->bcdDateTime.hundSecBCD != iTimestamp2Ptr->bcdDateTime.hundSecBCD)
    {
        /* if time 1 smaller than time 2 */
        if (FROM_BCD(iTimestamp1Ptr->bcdDateTime.hundSecBCD) < FROM_BCD(iTimestamp2Ptr->bcdDateTime.hundSecBCD))
        {
            return -1;
        }

        return 1;
    }

    if (iTimestamp1Ptr->milliSec != iTimestamp2Ptr->milliSec)
    {
        /* if time 1 smaller than time 2 */
        if (iTimestamp1Ptr->milliSec < iTimestamp2Ptr->milliSec)
        {
            return -1;
        }

        return 1;
    }

    /* if this point reached, then they match exactly */

    return 0;
}


/******************************************************************************
    @brief      This function determines if the requested date range
                times are valid.
                Note - very loose validation since times may jump around.

    @param      iPartId       - protected memory parition to operate on

    @return     TRUE if range is valid, otherwise FALSE

******************************************************************************/
static bool pmmif_ValidateRangeTimes(PMMIF_PARTITION_SEL_T iPartId)
{

    assert(iPartId < PMMIF_MAX_NUM_PARTITIONS);

    /* start must not be before end */
    if (pmmif_CmpTimestamp(&(mReadRangeData[iPartId].startTime), &(mReadRangeData[iPartId].endTime)) > 0)
    {
        UTIL_LogStr("pmmif_ValidateRangeTimes:Start time is after end time");
        return FALSE;
    }

    /* MWA 06-26-13: Don't check ranges against index / PMM.  Times may jump around and valid entries */
    /* may reside in the middle.  Will simply find the best info possible if dates jump around. */
    return TRUE;


}

static uint32 increment_block_num( uint32 block_num )
{
	uint32 return_block_num;

	return_block_num = block_num + 1;

	if ( return_block_num > 2047 )
	{
		return_block_num = 1; // skip 0 because it's the Bad Block Table
	}

	return return_block_num;
}

static uint32 decrement_block_num( uint32 block_num )
{
	uint32 return_block_num;

	return_block_num = block_num - 1;

	if ( return_block_num < 1 )
	{
		return_block_num = 2047;
	}

	return return_block_num;
}

/******************************************************************************
     @brief      This function determines the starting point for stored
                 event retrieval (oldest to newest) based on a date range.

     @param      iPartId       - protected memory partition to operate on
     @param      oEntryPtr     - if an entry is found this points to the entry in allocated memory
                                 if no entry found this is points to NULL
                                 ** THIS IS ALLOCATED MEMORY THAT NEEDS TO BE FREE'D BY THE CALLING FUNCTION **

     @return     TRUE if starting point found, otherwise FALSE

******************************************************************************/
static bool pmmif_FindReadRangeStart(int32 iThreadId, PMMIF_PARTITION_SEL_T iPartId, PMMIF_ENTRY_HDR_T **oEntryPtr)
{
    static char lTempBuf[200];

    PMMIF_ENTRY_HDR_T 		   *lStartEntryPtr;
    PMMIF_TIMESTAMP_T           lEntryTimestamp;
    PMM_MEM_SIZE_T              lSeekPoint;
    int                         lTimestampCmpResult;
    static char                 lTempDateBuf1[30];
    static char                 lTempDateBuf2[30];
    uint32 block_num          = 1;
    uint32 block_of_interest  = 0;
    uint32 page_of_interest   = 0;
    unsigned long   chip_num  = 0;
    unsigned long   page_num  = 0;
    unsigned long   event_num = 0;
    PMM_MEM_SIZE_T              tail_address;
    PMM_MEM_SIZE_T              read_address;
    t_location                  tail_location;
    t_location                  location;
    int                         count;

    assert(iPartId < PMMIF_MAX_NUM_PARTITIONS);

    /* Default to no entry found */
    *oEntryPtr = NULL;

    /* Validate the start and end times */
    if (TRUE != pmmif_ValidateRangeTimes(iPartId))
    {
   	    sprintf(lTempBuf, "pmmif_FindReadRangeStart:%d ValidateRangeTimes failed", __LINE__);
   	    UTIL_LogStr(lTempBuf);

        return FALSE;
    }

	/* Find the oldest entry and get the oldest timestamp */
    fs_ioctl(0, FS_IOCTL_GET_TL_ADDRESS, (int)&tail_address);

    if (FALSE == PMM_DRV_API_Seek(iPartId, tail_address))
    {
    	sprintf(lTempBuf, "Unable to seek to tail address (oldest entry)");
    	UTIL_LogStr(lTempBuf);

    	return FALSE;
    }

    lStartEntryPtr = pmmif_ReadEntry(iPartId);

	if (NULL == lStartEntryPtr)
	{
    	sprintf(lTempBuf, "Unable to read tail entry (oldest entry)");
    	UTIL_LogStr(lTempBuf);

    	return FALSE;
	}

	sprintf(lTempBuf, "Oldest entry address 0x%08lX, timestamp %s", tail_address, UTIL_GetHdrDateTimeStr(&(lStartEntryPtr->timeStamp.bcdDateTime), lTempDateBuf1));
	UTIL_LogStr(lTempBuf);

    /* Is the first event in the PMM in the requested range? */
    if ( pmmif_CmpTimestamp(&(mReadRangeData[iPartId].endTime), &(lStartEntryPtr->timeStamp)) < 0 )
    {
    	// The requested end time is before the first entry in the PMM
    	sprintf(lTempBuf, "EndTime %s is before first PMM entry %s",
    			UTIL_GetHdrDateTimeStr(&(mReadRangeData[iPartId].endTime.bcdDateTime), lTempDateBuf2),
				UTIL_GetHdrDateTimeStr(&(lStartEntryPtr->timeStamp.bcdDateTime), lTempDateBuf1));
    	UTIL_LogStr(lTempBuf);

    	free(lStartEntryPtr);

    	return FALSE;
    }

    /* Is the start time before or equal to the first PMM entry? */
    if ( pmmif_CmpTimestamp(&(mReadRangeData[iPartId].startTime), &(lStartEntryPtr->timeStamp)) <= 0 )
    {
    	// Return this event because the start time is before the first entry in the PMM
        *oEntryPtr = lStartEntryPtr;

    	sprintf(lTempBuf, "StartTime %s matches first entry %s",
    			UTIL_GetHdrDateTimeStr(&(mReadRangeData[iPartId].startTime.bcdDateTime), lTempDateBuf2),
				UTIL_GetHdrDateTimeStr(&(lStartEntryPtr->timeStamp.bcdDateTime), lTempDateBuf1));
    	UTIL_LogStr(lTempBuf);

    	return TRUE;
    }

    free(lStartEntryPtr);
    lStartEntryPtr = (PMMIF_ENTRY_HDR_T *)NULL;

    /* Determine which block the tail is in */
    address_to_location( &tail_location, tail_address);
	sprintf(lTempBuf, "tail offset 0x%08lX, block %4ld", tail_address, tail_location.block);
	UTIL_LogStr(lTempBuf);

    /* Already looked at the first event in the tail block - look at the first event in the NEXT block */
    block_num = increment_block_num( tail_location.block );

    chip_num = 0;
    page_num = 0;

    location.chip  = chip_num;
    location.block = block_num;
    location.page  = page_num;
    location.byte  = 0;

    /* Loop through each block except the first (it is the Bad Block Table) */
    for ( count = 1; count < 2047; count++ ) // limit this loop to 2047 times as that's the max number of blocks
    {
		location.block = block_num;
        location_to_address( &lSeekPoint, &location );

        /* Move to the beginning of the next block */
    	if (TRUE != PMM_DRV_API_Seek(iPartId, lSeekPoint))
    	{
			/* This means that the block we are trying to seek to is not between the head and the tail so it has likely not been written to, i.e. it is empty.
			 * Treat it like it is one block past the block that contains the start event */
    			/* block of interest should be in the previous block */

    		break;
    	}

    	/* Read the timestamp */
        lStartEntryPtr = pmmif_ReadEntry(iPartId);

    	if (NULL == lStartEntryPtr)
    	{
    		sprintf( lTempBuf, "pmmif_FindReadRangeStart: Unable to read entry/timestamp at offset 0x%08lX, block %4ld", lSeekPoint, block_num);
    		UTIL_LogStr(lTempBuf);

    		return FALSE;
    	}

    	lTimestampCmpResult = pmmif_CmpTimestamp(&(mReadRangeData[iPartId].startTime), &(lStartEntryPtr->timeStamp));

#if 0
		sprintf(lTempBuf, "block %4ld offset 0x%08lX EntryTime %s StartTime %s %d", block_num, lSeekPoint,
				UTIL_GetHdrDateTimeStr(&(lStartEntryPtr->timeStamp.bcdDateTime), lTempDateBuf1),
				UTIL_GetHdrDateTimeStr(&(mReadRangeData[iPartId].startTime.bcdDateTime), lTempDateBuf2), lTimestampCmpResult);
		UTIL_LogStr(lTempBuf);
#endif

		if (lTimestampCmpResult == 0)
		{
	    	// Return this event because the times match
	        *oEntryPtr = lStartEntryPtr;

	        sprintf(lTempBuf, "boi %4ld poi xxx event xxx offset 0x%08lX EntryTime %s StartTime %s", block_num, lSeekPoint,
	    			UTIL_GetHdrDateTimeStr(&(lStartEntryPtr->timeStamp.bcdDateTime), lTempDateBuf1),
	    			UTIL_GetHdrDateTimeStr(&(mReadRangeData[iPartId].startTime.bcdDateTime), lTempDateBuf2));
	    	UTIL_LogStr(lTempBuf);

	        return TRUE;
		}

		memcpy( &lEntryTimestamp, &(lStartEntryPtr->timeStamp), sizeof(PMMIF_TIMESTAMP_T));

		free( lStartEntryPtr );
	    lStartEntryPtr = (PMMIF_ENTRY_HDR_T *)NULL;

		if (lTimestampCmpResult < 0)
		{
			// The event is in the previous block
			block_of_interest = decrement_block_num( block_num );
#if 0
			sprintf( lTempBuf, "** Start block is in block %ld", block_of_interest );
			UTIL_LogStr(lTempBuf);
#endif
			break;
		}

	    block_num = increment_block_num( block_num );
    }

    if (block_of_interest == 0)
    {
		block_of_interest = decrement_block_num( block_num );
    }

    sprintf(lTempBuf, "boi %4ld poi xxx event xxx offset 0x%08lX EntryTime %s StartTime %s", block_of_interest, lSeekPoint,
			UTIL_GetHdrDateTimeStr(&(lEntryTimestamp.bcdDateTime), lTempDateBuf1),
			UTIL_GetHdrDateTimeStr(&(mReadRangeData[iPartId].startTime.bcdDateTime), lTempDateBuf2));
	UTIL_LogStr(lTempBuf);

	THREADMON_Kick( iThreadId );

	location.block = block_of_interest;

	/* Loop through each page */
    for ( page_num = 1; page_num <= 127; page_num++ ) // page 0 was read and tested above in the code that scanned blocks so start with page 1
    {
		location.page = page_num;
        location_to_address( &lSeekPoint, &location );

        /* Move to the beginning of the next page */
    	if (TRUE != PMM_DRV_API_Seek(iPartId, lSeekPoint))
    	{
			// The event is in the previous page
			// page_num is in the range 1 - 127 for this loop so don't have to check for page going from 0 to 127 prev page case
    		page_of_interest = page_num - 1;

			break;
    	}

    	/* Read the entry */
        lStartEntryPtr = pmmif_ReadEntry(iPartId);

    	if (NULL == lStartEntryPtr)
    	{
			// The event is in the previous page
			// page_num is in the range 1 - 127 for this loop so don't have to check for page going from 0 to 127 prev page case
			page_of_interest = page_num - 1;

    		break;
    	}

    	lTimestampCmpResult = pmmif_CmpTimestamp(&(mReadRangeData[iPartId].startTime), &lStartEntryPtr->timeStamp);

#if 0
		sprintf(lTempBuf, "block %4ld page %4ld offset 0x%08lX EntryTime %s StartTime %s %d", block_of_interest, page_num, lSeekPoint,
				UTIL_GetHdrDateTimeStr(&(lStartEntryPtr->timeStamp.bcdDateTime), lTempDateBuf1),
				UTIL_GetHdrDateTimeStr(&(mReadRangeData[iPartId].startTime.bcdDateTime), lTempDateBuf2), lTimestampCmpResult);
		UTIL_LogStr(lTempBuf);
#endif

		if (lTimestampCmpResult == 0)
		{
	    	// Return this event because the times match
	        *oEntryPtr = lStartEntryPtr;

	    	sprintf(lTempBuf, "boi %4ld poi %3ld event xxx offset 0x%08lX EntryTime %s StartTime %s",
	    			block_of_interest, page_num, lSeekPoint,
	    			UTIL_GetHdrDateTimeStr(&(lStartEntryPtr->timeStamp.bcdDateTime), lTempDateBuf1),
	    			UTIL_GetHdrDateTimeStr(&(mReadRangeData[iPartId].startTime.bcdDateTime), lTempDateBuf2));
	    	UTIL_LogStr(lTempBuf);

	        return TRUE;
		}

		memcpy( &lEntryTimestamp, &(lStartEntryPtr->timeStamp), sizeof(PMMIF_TIMESTAMP_T));

		free( lStartEntryPtr );
	    lStartEntryPtr = (PMMIF_ENTRY_HDR_T *)NULL;

		if (lTimestampCmpResult < 0)
		{
			// The event is in the previous page
			// page_num is in the range 1 - 127 for this loop so don't have to check for page going from 0 to 127 prev page case
			page_of_interest = page_num - 1;
#if 0
			sprintf( lTempBuf, "** Start page is in block %ld page %4ld", block_of_interest , page_of_interest);
			UTIL_LogStr(lTempBuf);
#endif
			break;
		}
		else
		{
			page_of_interest = page_num; // this happens if the event is in the last page (127)
		}
    }

	THREADMON_Kick( iThreadId );

	// Scan page for events

	location.block = block_of_interest;
	location.page  = page_of_interest;
    location_to_address( &lSeekPoint, &location );

    /* Move to the beginning of the block and page of interest */
	if (TRUE != PMM_DRV_API_Seek(iPartId, lSeekPoint))
	{
		sprintf(lTempBuf, "Unable to seek to offset 0x%08lX, block %4ld page% 4ld", lSeekPoint, block_num, page_num);
		UTIL_LogStr(lTempBuf);

		return FALSE;
	}

	// Search through the events

	// There should only be 240 events possible but give a little leeway (245) as 'event_num' is only used to keep this from looping forever
	for (event_num = 1; event_num <= 245; event_num++)
	{
    	/* Read the entry */
    	lStartEntryPtr = pmmif_ReadEntry(iPartId);

		if (NULL == lStartEntryPtr)
    	{
    		sprintf( lTempBuf, "Unable to read entry/timestamp at offset 0x%08lX, block %4ld page %4ld", lSeekPoint, block_num, page_num);
    		UTIL_LogStr(lTempBuf);

    		return FALSE;
    	}

		lTimestampCmpResult = pmmif_CmpTimestamp(&(mReadRangeData[iPartId].startTime), &(lStartEntryPtr->timeStamp));

        //fs_ioctl(0, FS_IOCTL_GET_RD_ADDRESS, (int)&read_address);
    	read_address = PMM_DRV_API_Tell( iPartId );

#if 0
		sprintf(lTempBuf, "boi %4ld poi %3ld event %3ld offset 0x%08lX/rd_loc 0x%08lX EntryTime %s StartTime %s %d",
				block_of_interest, page_of_interest, event_num, lSeekPoint, read_address,
				UTIL_GetHdrDateTimeStr(&(lStartEntryPtr->timeStamp.bcdDateTime), lTempDateBuf1),
				UTIL_GetHdrDateTimeStr(&(mReadRangeData[iPartId].startTime.bcdDateTime), lTempDateBuf2), lTimestampCmpResult);
		UTIL_LogStr(lTempBuf);
#endif

		if (lTimestampCmpResult <= 0)
		{
	    	// Return this event because the times match
	    	read_address = PMM_DRV_API_Tell( iPartId );

	        *oEntryPtr = lStartEntryPtr;

			sprintf(lTempBuf, "boi %4ld poi %3ld event %3ld rd_loc 0x%08lX EntryTime %s StartTime %s %d",
					block_of_interest, page_of_interest, event_num, read_address,
					UTIL_GetHdrDateTimeStr(&(lStartEntryPtr->timeStamp.bcdDateTime), lTempDateBuf1),
					UTIL_GetHdrDateTimeStr(&(mReadRangeData[iPartId].startTime.bcdDateTime), lTempDateBuf2), lTimestampCmpResult);
			UTIL_LogStr(lTempBuf);

			return TRUE;
		}

		free(lStartEntryPtr);
	    lStartEntryPtr = (PMMIF_ENTRY_HDR_T *)NULL;
	}

    return FALSE;
}

/******************************************************************************
    @brief      This function determines the starting point for stored
                event retrieval (oldest to newest) based on percentage of
                stored data. The file position is updated (via seek) to
                the start of the first event to be included.

    @param      iPartId       - protected memory parition to operate on

    @return     TRUE if starting point found, otherwise FALSE

******************************************************************************/
static bool pmmif_FindReadPctStart(PMMIF_PARTITION_SEL_T iPartId)
{
    uint32					   pmm_head;
    uint32                     pmm_tail;
    uint32                     pmm_total_pages;
    uint32                     head_page_index;
    uint32                     mid_page_index;
    uint32                     tail_page_index;
    uint32                     delta;

    assert(iPartId < PMMIF_MAX_NUM_PARTITIONS);

    pmm_head        = PMM_DRV_API_GetHeadOffset(iPartId);
    pmm_tail        = PMM_DRV_API_GetTailOffset(iPartId);
    pmm_total_pages = PMM_DRV_API_GetPartitionSize(iPartId);

    head_page_index = address_to_index(pmm_head);
    tail_page_index = address_to_index(pmm_tail);

    if (0 == mReadLogPct[iPartId])
    {
        return FALSE;
    }


    if( head_page_index > tail_page_index)
    {
    	// This is normal, not wrap-around situation
    	delta = head_page_index - tail_page_index;
    	mid_page_index = (tail_page_index + (100 - mReadLogPct[iPartId]) * delta / 100) % pmm_total_pages;
    	PMM_DRV_API_Seek(iPartId, index_to_address(mid_page_index));
    }
    else
    {
    	// There has been a wrap-around
    	delta = (pmm_total_pages - tail_page_index) + head_page_index;
    	mid_page_index = (tail_page_index + (100 - mReadLogPct[iPartId]) * delta / 100) % pmm_total_pages;
    	PMM_DRV_API_Seek(iPartId, index_to_address(mid_page_index));
    }

    return TRUE;
}


/******************************************************************************
    @brief      This function returns the available time range in PMM

    @param      none

    @return     returns the Timestamp of the Newest entry available with PMM module

******************************************************************************/

PMMIF_READ_RANGE_T PMMIF_GetAvailablePMMTimeRange(PMMIF_PARTITION_SEL_T iPartId)
{
    static char lTempBuf[200];
    const PMMIDX_INDEX_ENTRY_T *lIndexEntryPtr;
    static PMMIF_READ_RANGE_T  lAvailableDateRangeWithChmm;

    assert(iPartId < PMMIF_MAX_NUM_PARTITIONS);

    if (TRUE != PMM_DRV_API_SetPMMReadMode(iPartId, APP_GetMyPID()))
    {
        UTIL_LogStr("PMMIF_GetPMMTimeRange:Setting read mode failed");
    }

    APP_SetState(CHMM_STATE_OFFLINE, __FILE__, __LINE__);

    if (TRUE != PMMIDX_LoadIndex(iPartId))
    {
        memset(&lAvailableDateRangeWithChmm, 0, sizeof(lAvailableDateRangeWithChmm));
        return lAvailableDateRangeWithChmm;
    }

    /* get pointer to oldest entry in index */
    lIndexEntryPtr = PMMIDX_GetOldestIndexEntry(iPartId);
    if (NULL == lIndexEntryPtr)
    {
        sprintf(lTempBuf, "PMMIF_GetPMMTimeRange:Unable to get oldest index entry at line %d", __LINE__);
        UTIL_LogStr(lTempBuf);
    }
    /* move to the associated event entry in the protected memory */
    if (TRUE != PMM_DRV_API_Seek(iPartId, lIndexEntryPtr->entryOffset))
    {
        sprintf(lTempBuf, "PMMIF_GetPMMTimeRange:Unable to seek at line %d", __LINE__);
        UTIL_LogStr(lTempBuf);
    }
    /* extract the PMMIF timestamp from the entry at the
       current file position    */
    if (TRUE != pmmif_ReadEntryAndGetPMMIFTimestamp(iPartId, &lAvailableDateRangeWithChmm.startTime))
    {
        sprintf(lTempBuf, "PMMIF_GetPMMTimeRange:Unable to read entry at line %d", __LINE__);
        UTIL_LogStr(lTempBuf);
    }
    // Update the Oldest time stamp available with CHMM
    //mAvailableDateRangeWithChmm.startTime = lOldestEntryAppTimestamp;
    lIndexEntryPtr = PMMIDX_GetNewestIndexEntry(iPartId);
    if (NULL == lIndexEntryPtr)
    {
        sprintf(lTempBuf, "PMMIF_GetPMMTimeRange:Unable to get newest index entry at line %d", __LINE__);
        UTIL_LogStr(lTempBuf);
    }
    /* move to the associated event entry in the protected memory */
    if (TRUE != PMM_DRV_API_Seek(iPartId, lIndexEntryPtr->entryOffset))
    {
        sprintf(lTempBuf, "PMMIF_GetPMMTimeRange:Unable to seek at line %d", __LINE__);
        UTIL_LogStr(lTempBuf);
    }
    /* extract the PMMIF timestamp from the entry at the
       current file position     */
    if (TRUE != pmmif_ReadEntryAndGetPMMIFTimestamp(iPartId, &lAvailableDateRangeWithChmm.endTime))
    {
        sprintf(lTempBuf, "PMMIF_GetPMMTimeRange:Unable to read entry at line %d", __LINE__);
        UTIL_LogStr(lTempBuf);
    }

    //Clear the read mode of PMM
    (void) PMMIF_EndEventRead(iPartId);
    // Update the Newest time stamp available with CHMM
    //mAvailableDateRangeWithChmm.endTime = lNewestEntryAppTimestamp;

    return lAvailableDateRangeWithChmm;
}


/****************************************************************************
   @brief       This function extracts and validates the encrypted parameter
                value in the provided string. If the value is not valid
                FALSE is returned, otherwise TRUE is returned

   @return      TRUE if value is valid, otherwise FALSE

 ******************************************************************************/
static bool pmmif_GetEncryptedParam(uint32 iSerialNo, const char *iEncryptedText, uint8 iValueId, uint32 *iRetValue)
{
    char   lEncryptedValueStr[9];
    char   lCRCValueStr[9];
    uint32 lPlainTextValue;
    uint32 lEncryptedValue;
    uint32 lCRCValue;
    uint32 lCRCData[3];

    memset(lEncryptedValueStr, 0, sizeof(lEncryptedValueStr));
    memset(lCRCValueStr, 0, sizeof(lCRCValueStr));

    memcpy(lEncryptedValueStr, iEncryptedText, 8);
    memcpy(lCRCValueStr, &iEncryptedText[8], 8);

    sscanf(lEncryptedValueStr, "%lx", &lEncryptedValue);
    sscanf(lCRCValueStr, "%lx", &lCRCValue);

    lPlainTextValue = PARAMCRYPTO_Decrypt(iSerialNo, lEncryptedValue, iValueId);

    lCRCData[0] = iSerialNo;
    lCRCData[1] = lPlainTextValue;
    lCRCData[2] = lEncryptedValue;

    if (lCRCValue == CRC_CalcCRC32(0xFFFFFFFF, lCRCData, sizeof(lCRCData)))
    {
        *iRetValue = lPlainTextValue;
        return TRUE;
    }

    return FALSE;

}


/****************************************************************************
   @brief       This function loads hw revision, serial no. etc. from
                .ini file

   @return      none

 ******************************************************************************/
static void pmmif_LoadPMMInfo(void)
{
    dictionary *lpsDictionary  = NULL;
    char       *lpszFieldValue = NULL;
    uint32     lSerialNo;

    lSerialNo = atoi(APP_GetHWSerialNumStr());

    /* Load the .ini file into a dictionary for use by the iniparser functions */
    lpsDictionary = iniparser_load("/tmp/eeprom.bin");

    /* If the dictionary can't be loaded, then return */
    if (lpsDictionary == NULL)
    {
        return;
    }

    lpszFieldValue = iniparser_getstring(lpsDictionary, (char *) gINIKeyTable[PMMIF_INIFLD_MFG_INSTALLED_MEM], NULL);

    /* if the field exists, save the data. */
    if (lpszFieldValue != NULL)
    {
        if (TRUE != pmmif_GetEncryptedParam(lSerialNo, lpszFieldValue, 1, &mHwInstalledMemGiB))
        {
            mHwInstalledMemGiB = 0;
        }
    }


    lpszFieldValue = iniparser_getstring(lpsDictionary, (char *) gINIKeyTable[PMMIF_INIFLD_MFG_ENABLED_MEM], NULL);

    /* if the field exists, save the data. */
    if (lpszFieldValue != NULL)
    {
        if (TRUE != pmmif_GetEncryptedParam(lSerialNo, lpszFieldValue, 2, &mHwEnabledMemGiB))
        {
            mHwEnabledMemGiB = 0;
        }
    }

    /* free the dynamically allocated dictionary memory */
    iniparser_freedict(lpsDictionary);
}
