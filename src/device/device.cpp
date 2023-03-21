#include "device/device.hpp"

LOG_MODULE_REGISTER(device, LOG_LEVEL_DBG);

Device::Device()

{
    {
        /* Default communication configuration. We use default non-STS DW mode. */
        config.chan = 5;                      // Channel number.
        config.txPreambLength = DWT_PLEN_128; // Preamble length. Used in TX only.
        config.rxPAC = DWT_PAC8;              // Preamble acquisition chunk size. Used in RX only.
        config.txCode = 9;                    // TX preamble code. Used in TX only.
        config.rxCode = 9;                    // RX preamble code. Used in RX only.
        config.sfdType = DWT_SFD_DW_8;        // use non-standard 8 symbol
        config.dataRate = DWT_BR_6M8;         // Data rate.
        config.phrMode = DWT_PHRMODE_STD;     // PHY header mode.
        config.phrRate = DWT_PHRRATE_STD;     // PHY header rate.
        config.sfdTO = (129 + 8 - 8);         // SFD timeout Used in RX only.
                                              // (preamble length + 1 + SFD length - PAC size).
        config.stsMode = DWT_STS_MODE_OFF;    // STS disabled
        config.stsLength = DWT_STS_LEN_64;    // STS length see allowed values in Enum dwt_sts_lengths_e
        config.pdoaMode = DWT_PDOA_M0;        // PDOA mode off
    }
    {
        dw3000_hw_init();
        dw3000_hw_reset();
        dw3000_hw_init_interrupt();

        dw3000_spi_speed_fast();

        if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) == DWT_ERROR)
        {
            LOG_DBG("DEV Probe FAILED");
        }
        while (!dwt_checkidlerc())
            ;
        if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
        {
            LOG_DBG("DEV INIT FAILED");
        }
        if (dwt_configure(&config))
        {
            LOG_DBG("DEV CONFIG FAILED");
        }
        else
            LOG_DBG("DEV CONFIG SUCCEED");

        k_sleep(K_SECONDS(1));

        dwt_configuretxrf(&txconfig_options);

        dwt_setrxantennadelay(RX_ANT_DLY); // set RX antenna delay time
        dwt_settxantennadelay(TX_ANT_DLY); // set TX antenna delay time

        dwt_setrxaftertxdelay(0);
        dwt_setrxtimeout(0);

        // dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

        dwt_setinterrupt(DWT_INT_TXFRS_BIT_MASK | DWT_INT_RXFCG_BIT_MASK, 0, DWT_ENABLE_INT);
    }
}

void Device::app(void *, void *, void *)
{
}

void Device::set_callback()
{
    dwt_setcallbacks(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
}

bool Device::init_dw3000()
{

    this->set_callback();

    dw3000_hw_interrupt_enable();

    return true;
}

void Device::tx_done_cb(const dwt_cb_data_t *cb_data) {}
void Device::rx_ok_cb(const dwt_cb_data_t *cb_data) {}
void Device::rx_timeout_cb(const dwt_cb_data_t *cb_data) {}
void Device::rx_err_cb(const dwt_cb_data_t *cb_data) {}
void Device::spi_err_cb(const dwt_cb_data_t *cb_data) {}
void Device::spi_ready_cb(const dwt_cb_data_t *cb_data) {}
void Device::dual_spi_cb(const dwt_cb_data_t *cb_data) {}
