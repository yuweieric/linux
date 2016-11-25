#include <linux/printk.h>                                                       // pr_err, printk, etc
#include "fusb30x_global.h"                                                     // Chip structure
#include "platform_helpers.h"                                                   // Implementation details
#include "../core/platform.h"
#include "../core/PD_Types.h"
#include "../core/TypeC_Types.h"
#include <linux/hisi/usb/hisi_pd_dev.h>
#include <linux/delay.h>

static FSC_BOOL g_vbus = 0;

//open vbus and switch to host
extern FSC_BOOL			PolicyHasContract;
extern doDataObject_t	USBPDContract;
extern USBTypeCCurrent	SourceCurrent;
static void fusb_typec_open_vbus(FSC_BOOL blnEnable)
{
	if (g_vbus != blnEnable)
	{
		struct pd_dpm_vbus_state vbus_state;
		if (TRUE == blnEnable)
		{
			vbus_state.mv = 5000;
			if (PolicyHasContract)
			{
				vbus_state.vbus_type = TCP_VBUS_CTRL_PD_DETECT;
				vbus_state.ma = USBPDContract.FVRDO.OpCurrent * 10;
			}
			else
			{
				vbus_state.vbus_type = 0;
				switch (SourceCurrent)
				{
					case utccDefault:
						vbus_state.ma = 900;
						break;
					case utcc1p5A:
						vbus_state.ma = 1500;
						break;
					case utcc3p0A:
						vbus_state.ma = 3000;
						break;
					case utccNone:
						vbus_state.ma = 0;
						break;
				}
			}
			pd_dpm_handle_pe_event(PD_DPM_PE_EVT_SOURCE_VBUS, (void *)&vbus_state);
		}
		else
		{
			pd_dpm_handle_pe_event(PD_DPM_PE_EVT_DIS_VBUS_CTRL, NULL);
		}
	}
}

/*******************************************************************************
* Function:        platform_set/get_vbus_lvl_enable
* Input:           VBUS_LVL - requested voltage
*                  Boolean - enable this voltage level
*                  Boolean - turn off other supported voltages
* Return:          Boolean - on or off
* Description:     Provide access to the VBUS control pins.
******************************************************************************/
void platform_set_vbus_lvl_enable(VBUS_LVL level, FSC_BOOL blnEnable, FSC_BOOL blnDisableOthers)
{
    FSC_U32 i;

    // Additional VBUS levels can be added here as needed.
    switch (level)
    {
    case VBUS_LVL_5V:
        // Enable/Disable the 5V Source
		pr_info("FUSB  %s - %s typeC 5V VBUS (g_vbus = %s) ...\n", __func__, blnEnable ? "enabling" : "disabling", g_vbus ? "TRUE" : "FALSE");
        fusb_typec_open_vbus(blnEnable);
		g_vbus = blnEnable;
        break;
    case VBUS_LVL_12V:
        // Enable/Disable the 12V Source
        break;
    default:
        // Otherwise, do nothing.
        break;
    }

    // Turn off other levels, if requested
    if (blnDisableOthers || ((level == VBUS_LVL_ALL) && (blnEnable == FALSE)))
    {
        i = 0;

        do {
            // Skip the current level
            if( i == level ) continue;

            // Turn off the other level(s)
            platform_set_vbus_lvl_enable( i, FALSE, FALSE );
        } while (++i < VBUS_LVL_COUNT);
    }

    return;
}

FSC_BOOL platform_get_vbus_lvl_enable(VBUS_LVL level)
{
    // Additional VBUS levels can be added here as needed.
    switch (level)
    {
    case VBUS_LVL_5V:
        // Return the state of the 5V VBUS Source.
        return g_vbus;

    case VBUS_LVL_12V:
        // Return the state of the 12V VBUS Source.

    default:
        // Otherwise, return FALSE.
        return FALSE;
    }
}

/*******************************************************************************
* Function:        platform_set_vbus_discharge
* Input:           Boolean
* Return:          None
* Description:     Enable/Disable Vbus Discharge Path
******************************************************************************/
void platform_set_vbus_discharge(FSC_BOOL blnEnable)
{
    // TODO - Implement if required for platform
}

/*******************************************************************************
* Function:        platform_get_device_irq_state
* Input:           None
* Return:          Boolean.  TRUE = Interrupt Active
* Description:     Get the state of the INT_N pin.  INT_N is active low.  This
*                  function handles that by returning TRUE if the pin is
*                  pulled low indicating an active interrupt signal.
******************************************************************************/
FSC_BOOL platform_get_device_irq_state(void)
{
    return fusb_GPIO_Get_IntN() ? TRUE : FALSE;
}

/*******************************************************************************
* Function:        platform_i2c_write
* Input:           SlaveAddress - Slave device bus address
*                  RegAddrLength - Register Address Byte Length
*                  DataLength - Length of data to transmit
*                  PacketSize - Maximum size of each transmitted packet
*                  IncSize - Number of bytes to send before incrementing addr
*                  RegisterAddress - Internal register address
*                  Data - Buffer of char data to transmit
* Return:          Error state
* Description:     Write a char buffer to the I2C peripheral.
******************************************************************************/
FSC_BOOL platform_i2c_write(FSC_U8 SlaveAddress,
                        FSC_U8 RegAddrLength,
                        FSC_U8 DataLength,
                        FSC_U8 PacketSize,
                        FSC_U8 IncSize,
                        FSC_U32 RegisterAddress,
                        FSC_U8* Data)
{
    FSC_BOOL ret = FALSE;
    if (Data == NULL)
    {
        pr_err("%s - Error: Write data buffer is NULL!\n", __func__);
        ret = FALSE;
    }
    else if (fusb_I2C_WriteData((FSC_U8)RegisterAddress, DataLength, Data))
    {
        ret = TRUE;
    }
    else  // I2C Write failure
    {
        ret = FALSE;       // Write data block to the device
    }
    return ret;
}

/*******************************************************************************
* Function:        platform_i2c_read
* Input:           SlaveAddress - Slave device bus address
*                  RegAddrLength - Register Address Byte Length
*                  DataLength - Length of data to attempt to read
*                  PacketSize - Maximum size of each received packet
*                  IncSize - Number of bytes to recv before incrementing addr
*                  RegisterAddress - Internal register address
*                  Data - Buffer for received char data
* Return:          Error state.
* Description:     Read char data from the I2C peripheral.
******************************************************************************/
FSC_BOOL platform_i2c_read(FSC_U8 SlaveAddress,
                       FSC_U8 RegAddrLength,
                       FSC_U8 DataLength,
                       FSC_U8 PacketSize,
                       FSC_U8 IncSize,
                       FSC_U32 RegisterAddress,
                       FSC_U8* Data)
{
    FSC_BOOL ret = FALSE;
    FSC_S32 i = 0;
    FSC_U8 temp = 0;
    struct fusb30x_chip* chip = fusb30x_GetChip();
    if (!chip)
    {
        pr_err("FUSB  %s - Error: Chip structure is NULL!\n", __func__);
        return FALSE;
    }

    if (Data == NULL)
    {
        pr_err("%s - Error: Read data buffer is NULL!\n", __func__);
        ret = FALSE;
    }
    else if (DataLength > 1 && chip->use_i2c_blocks)    // Do block reads if able and necessary
    {
        if (!fusb_I2C_ReadBlockData(RegisterAddress, DataLength, Data))
        {
            ret = FALSE;
			pr_err("%s - Error: I2C Read Error!\n", __func__);
        }
        else
        {
            ret = TRUE;
        }
    }
    else
    {
        for (i = 0; i < DataLength; i++)
        {
            if (fusb_I2C_ReadData((FSC_U8)RegisterAddress + i, &temp))
            {
                Data[i] = temp;
                ret = TRUE;
            }
            else
            {
                ret = FALSE;
                break;
            }
        }
    }

    return ret;
}

/*****************************************************************************
* Function:        platform_enable_timer
* Input:           enable - TRUE to enable platform timer, FALSE to disable
* Return:          None
* Description:     Enables or disables platform timer
******************************************************************************/
void platform_enable_timer(FSC_BOOL enable)
{
    if (enable == TRUE)
    {
        fusb_StartTimers();
    }
    else
    {
        fusb_StopTimers();
    }
}

/*****************************************************************************
* Function:        platform_delay_10us
* Input:           delayCount - Number of 10us delays to wait
* Return:          None
* Description:     Perform a software delay in intervals of 10us.
******************************************************************************/
void platform_delay_10us(FSC_U32 delayCount)
{
    fusb_Delay10us(delayCount);
}

/*******************************************************************************
* Function:        platform_notify_cc_orientation
* Input:           orientation - Orientation of CC (NONE, CC1, CC2)
* Return:          None
* Description:     A callback used by the core to report to the platform the
*                  current CC orientation. Called in SetStateAttached... and
*                  SetStateUnattached functions.
******************************************************************************/
extern SourceOrSink		sourceOrSink;							// are we currently a source or a sink
extern CCTermType		CC1TermPrevious, CC2TermPrevious;		// cc line status
extern CCTermType		VCONNTerm;								// vconn line status
void platform_notify_cc_orientation(CC_ORIENTATION orientation)
{
	// Optional: Notify platform of CC orientation
	FSC_U8 cc1State, cc2State;
	struct pd_dpm_typec_state tc_state;
	tc_state.polarity = (orientation == CC2) ? TCPC_CTRL_PLUG_ORIENTATION : 0;
    pr_info("FUSB  %s - orientation = %d, sourceOrSink = %s\n", __func__, orientation, (sourceOrSink == SOURCE) ? "SOURCE" : "SINK");
	if (orientation != NONE)
	{
		if (sourceOrSink == SOURCE)
		{
			tc_state.new_state = PD_DPM_TYPEC_ATTACHED_SRC;
		}
		else
		{
			tc_state.new_state = PD_DPM_TYPEC_ATTACHED_SNK;
		}
	}
	else if (orientation == NONE)
	{
		tc_state.new_state = PD_DPM_TYPEC_UNATTACHED;
	}

	// Optional: Notify platform of CC pull-up/pull-down status
	switch (orientation)
	{
		case NONE:
			cc1State = 0;
			cc2State = 0;
			break;
		case CC1:
			cc1State = CC1TermPrevious;
			cc2State = VCONNTerm;
			break;
		case CC2:
			cc1State = VCONNTerm;
			cc2State = CC2TermPrevious;
			break;
	}
	// cc1State & cc2State = pull-up or pull-down status of each CC line:
	// CCTypeOpen		= 0
	// CCTypeRa		= 1
	// CCTypeRdUSB		= 2
	// CCTypeRd1p5		= 3
	// CCTypeRd3p0		= 4
	// CCTypeUndefined	= 5
	tc_state.cc1_status = cc1State;
	tc_state.cc2_status = cc2State;
	pd_dpm_handle_pe_event(PD_DPM_PE_EVT_TYPEC_STATE, (void*)&tc_state);
}

/*******************************************************************************
* Function:        platform_notify_pd_contract
* Input:           contract - TRUE: Contract, FALSE: No Contract
*				   PDvoltage - PD contract voltage in 50mV steps
*				   PDcurrent - PD contract current in 10mA steps
* Return:          None
* Description:     A callback used by the core to report to the platform the
*                  current PD contract status. Called in PDPolicy.
*******************************************************************************/
void platform_notify_pd_contract(FSC_BOOL contract, FSC_U32 PDvoltage, FSC_U32 PDcurrent)
{
    // Optional: Notify platform of PD contract
	pr_info("FUSB  %s - PD Contract: %dmV/%dmA; sourceOrSink: %s\n", __func__, PDvoltage * 50, PDcurrent *10, (sourceOrSink == SOURCE) ? "SOURCE" : "SINK");
	if (contract == TRUE)
	{
		struct pd_dpm_vbus_state vbus_state;
		vbus_state.vbus_type = TCP_VBUS_CTRL_PD_DETECT;
		vbus_state.mv = PDvoltage * 50;
		vbus_state.ma = PDcurrent * 10;
		if (sourceOrSink == SINK)
		{
			pd_dpm_handle_pe_event(PD_DPM_PE_EVT_SINK_VBUS, (void *)&vbus_state);
		}
	}
//	else
//	{
//		pd_dpm_handle_pe_event(PD_DPM_PE_EVT_DIS_VBUS_CTRL, NULL);
//	}
}

/*******************************************************************************
* Function:        platform_notify_pd_state
* Input:           state - SOURCE or SINK
* Return:          None
* Description:     A callback used by the core to report to the platform the
*                  PD state status when entering SOURCE or SINK. Called in PDPolicy.
*******************************************************************************/
void platform_notify_pd_state(SourceOrSink state)
{
    // Optional: Notify platform of PD SRC or SNK state
    struct pd_dpm_pd_state pd_state;
	pd_state.connected = (state == SOURCE) ? PD_CONNECT_PE_READY_SRC : PD_CONNECT_PE_READY_SNK;
	pd_dpm_handle_pe_event(PD_DPM_PE_EVT_PD_STATE, (void *)&pd_state);
}

/*******************************************************************************
* Function:        platform_notify_unsupported_accessory
* Input:           None
* Return:          None
* Description:     A callback used by the core to report entry to the
*                  Unsupported Accessory state. The platform may implement
*                  USB Billboard.
*******************************************************************************/
void platform_notify_unsupported_accessory(void)
{
	// Optional: Notify platform of unsupported accessory
}

/*******************************************************************************
* Function:        platform_notify_data_role
* Input:           PolicyIsDFP - Current data role
* Return:          None
* Description:     A callback used by the core to report the new data role after
*                  a data role swap.
*******************************************************************************/
void platform_notify_data_role(FSC_BOOL PolicyIsDFP)
{
	// Optional: Notify platform of data role change
    struct pd_dpm_swap_state swap_state;
	swap_state.new_role = (sourceOrSink == SOURCE) ? PD_ROLE_DFP : PD_ROLE_UFP;
    pd_dpm_handle_pe_event(PD_DPM_PE_EVT_DR_SWAP, (void *)&swap_state);
}
