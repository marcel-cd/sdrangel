/**
 * SDRangel
 * This is the web REST/JSON API of SDRangel SDR software. SDRangel is an Open Source Qt5/OpenGL 3.0+ (4.3+ in Windows) GUI and server Software Defined Radio and signal analyzer in software. It supports Airspy, BladeRF, HackRF, LimeSDR, Zcuadrv9009, RTL-SDR, SDRplay RSP1 and FunCube    ---   Limitations and specifcities:    * In SDRangel GUI the first Rx device set cannot be deleted. Conversely the server starts with no device sets and its number of device sets can be reduced to zero by as many calls as necessary to /sdrangel/deviceset with DELETE method.   * Preset import and export from/to file is a server only feature.   * Device set focus is a GUI only feature.   * The following channels are not implemented (status 501 is returned): ATV and DATV demodulators, Channel Analyzer NG, LoRa demodulator   * The device settings and report structures contains only the sub-structure corresponding to the device type. The DeviceSettings and DeviceReport structures documented here shows all of them but only one will be or should be present at a time   * The channel settings and report structures contains only the sub-structure corresponding to the channel type. The ChannelSettings and ChannelReport structures documented here shows all of them but only one will be or should be present at a time    ---
 *
 * OpenAPI spec version: 7.0.0
 * Contact: f4exb06@gmail.com
 *
 * NOTE: This class is auto generated by the swagger code generator program.
 * https://github.com/swagger-api/swagger-codegen.git
 * Do not edit the class manually.
 */

/*
 * SWGZcuadrv9009InputSettings.h
 *
 * Zcuadrv9009
 */

#ifndef SWGZcuadrv9009InputSettings_H_
#define SWGZcuadrv9009InputSettings_H_

#include <QJsonObject>


#include <QString>

#include "SWGObject.h"
#include "export.h"

namespace SWGSDRangel {

class SWG_API SWGZcuadrv9009InputSettings: public SWGObject {
public:
    SWGZcuadrv9009InputSettings();
    SWGZcuadrv9009InputSettings(QString* json);
    virtual ~SWGZcuadrv9009InputSettings();
    void init();
    void cleanup();

    virtual QString asJson () override;
    virtual QJsonObject* asJsonObject() override;
    virtual void fromJsonObject(QJsonObject &json) override;
    virtual SWGZcuadrv9009InputSettings* fromJson(QString &jsonString) override;

    qint64 getCenterFrequency();
    void setCenterFrequency(qint64 center_frequency);

    qint32 getDevSampleRate();
    void setDevSampleRate(qint32 dev_sample_rate);

    qint32 getLOppmTenths();
    void setLOppmTenths(qint32 l_oppm_tenths);

    qint32 getLpfFirEnable();
    void setLpfFirEnable(qint32 lpf_fir_enable);

    qint32 getLpfFirbw();
    void setLpfFirbw(qint32 lpf_firbw);

    qint32 getLpfFiRlog2Decim();
    void setLpfFiRlog2Decim(qint32 lpf_fi_rlog2_decim);

    qint32 getLpfFirGain();
    void setLpfFirGain(qint32 lpf_fir_gain);

    qint32 getFcPos();
    void setFcPos(qint32 fc_pos);

    qint32 getDcBlock();
    void setDcBlock(qint32 dc_block);

    qint32 getIqCorrection();
    void setIqCorrection(qint32 iq_correction);

    qint32 getHwBbdcBlock();
    void setHwBbdcBlock(qint32 hw_bbdc_block);

    qint32 getHwRfdcBlock();
    void setHwRfdcBlock(qint32 hw_rfdc_block);

    qint32 getHwIqCorrection();
    void setHwIqCorrection(qint32 hw_iq_correction);

    qint32 getLog2Decim();
    void setLog2Decim(qint32 log2_decim);

    qint32 getLpfBw();
    void setLpfBw(qint32 lpf_bw);

    qint32 getGain();
    void setGain(qint32 gain);

    qint32 getAntennaPath();
    void setAntennaPath(qint32 antenna_path);

    qint32 getGainMode();
    void setGainMode(qint32 gain_mode);

    qint32 getTransverterMode();
    void setTransverterMode(qint32 transverter_mode);

    qint64 getTransverterDeltaFrequency();
    void setTransverterDeltaFrequency(qint64 transverter_delta_frequency);

    qint32 getIqOrder();
    void setIqOrder(qint32 iq_order);

    qint32 getUseReverseApi();
    void setUseReverseApi(qint32 use_reverse_api);

    QString* getReverseApiAddress();
    void setReverseApiAddress(QString* reverse_api_address);

    qint32 getReverseApiPort();
    void setReverseApiPort(qint32 reverse_api_port);

    qint32 getReverseApiDeviceIndex();
    void setReverseApiDeviceIndex(qint32 reverse_api_device_index);


    virtual bool isSet() override;

private:
    qint64 center_frequency;
    bool m_center_frequency_isSet;

    qint32 dev_sample_rate;
    bool m_dev_sample_rate_isSet;

    qint32 l_oppm_tenths;
    bool m_l_oppm_tenths_isSet;

    qint32 lpf_fir_enable;
    bool m_lpf_fir_enable_isSet;

    qint32 lpf_firbw;
    bool m_lpf_firbw_isSet;

    qint32 lpf_fi_rlog2_decim;
    bool m_lpf_fi_rlog2_decim_isSet;

    qint32 lpf_fir_gain;
    bool m_lpf_fir_gain_isSet;

    qint32 fc_pos;
    bool m_fc_pos_isSet;

    qint32 dc_block;
    bool m_dc_block_isSet;

    qint32 iq_correction;
    bool m_iq_correction_isSet;

    qint32 hw_bbdc_block;
    bool m_hw_bbdc_block_isSet;

    qint32 hw_rfdc_block;
    bool m_hw_rfdc_block_isSet;

    qint32 hw_iq_correction;
    bool m_hw_iq_correction_isSet;

    qint32 log2_decim;
    bool m_log2_decim_isSet;

    qint32 lpf_bw;
    bool m_lpf_bw_isSet;

    qint32 gain;
    bool m_gain_isSet;

    qint32 antenna_path;
    bool m_antenna_path_isSet;

    qint32 gain_mode;
    bool m_gain_mode_isSet;

    qint32 transverter_mode;
    bool m_transverter_mode_isSet;

    qint64 transverter_delta_frequency;
    bool m_transverter_delta_frequency_isSet;

    qint32 iq_order;
    bool m_iq_order_isSet;

    qint32 use_reverse_api;
    bool m_use_reverse_api_isSet;

    QString* reverse_api_address;
    bool m_reverse_api_address_isSet;

    qint32 reverse_api_port;
    bool m_reverse_api_port_isSet;

    qint32 reverse_api_device_index;
    bool m_reverse_api_device_index_isSet;

};

}

#endif /* SWGZcuadrv9009InputSettings_H_ */
