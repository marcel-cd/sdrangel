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


#include "SWGZcuadrv9009InputSettings.h"

#include "SWGHelpers.h"

#include <QJsonDocument>
#include <QJsonArray>
#include <QObject>
#include <QDebug>

namespace SWGSDRangel {

SWGZcuadrv9009InputSettings::SWGZcuadrv9009InputSettings(QString* json) {
    init();
    this->fromJson(*json);
}

SWGZcuadrv9009InputSettings::SWGZcuadrv9009InputSettings() {
    center_frequency = 0L;
    m_center_frequency_isSet = false;
    dev_sample_rate = 0;
    m_dev_sample_rate_isSet = false;
    l_oppm_tenths = 0;
    m_l_oppm_tenths_isSet = false;
    lpf_fir_enable = 0;
    m_lpf_fir_enable_isSet = false;
    lpf_firbw = 0;
    m_lpf_firbw_isSet = false;
    lpf_fi_rlog2_decim = 0;
    m_lpf_fi_rlog2_decim_isSet = false;
    lpf_fir_gain = 0;
    m_lpf_fir_gain_isSet = false;
    fc_pos = 0;
    m_fc_pos_isSet = false;
    dc_block = 0;
    m_dc_block_isSet = false;
    iq_correction = 0;
    m_iq_correction_isSet = false;
    hw_bbdc_block = 0;
    m_hw_bbdc_block_isSet = false;
    hw_rfdc_block = 0;
    m_hw_rfdc_block_isSet = false;
    hw_iq_correction = 0;
    m_hw_iq_correction_isSet = false;
    log2_decim = 0;
    m_log2_decim_isSet = false;
    lpf_bw = 0;
    m_lpf_bw_isSet = false;
    gain = 0;
    m_gain_isSet = false;
    antenna_path = 0;
    m_antenna_path_isSet = false;
    gain_mode = 0;
    m_gain_mode_isSet = false;
    transverter_mode = 0;
    m_transverter_mode_isSet = false;
    transverter_delta_frequency = 0L;
    m_transverter_delta_frequency_isSet = false;
    iq_order = 0;
    m_iq_order_isSet = false;
    use_reverse_api = 0;
    m_use_reverse_api_isSet = false;
    reverse_api_address = nullptr;
    m_reverse_api_address_isSet = false;
    reverse_api_port = 0;
    m_reverse_api_port_isSet = false;
    reverse_api_device_index = 0;
    m_reverse_api_device_index_isSet = false;
}

SWGZcuadrv9009InputSettings::~SWGZcuadrv9009InputSettings() {
    this->cleanup();
}

void
SWGZcuadrv9009InputSettings::init() {
    center_frequency = 0L;
    m_center_frequency_isSet = false;
    dev_sample_rate = 0;
    m_dev_sample_rate_isSet = false;
    l_oppm_tenths = 0;
    m_l_oppm_tenths_isSet = false;
    lpf_fir_enable = 0;
    m_lpf_fir_enable_isSet = false;
    lpf_firbw = 0;
    m_lpf_firbw_isSet = false;
    lpf_fi_rlog2_decim = 0;
    m_lpf_fi_rlog2_decim_isSet = false;
    lpf_fir_gain = 0;
    m_lpf_fir_gain_isSet = false;
    fc_pos = 0;
    m_fc_pos_isSet = false;
    dc_block = 0;
    m_dc_block_isSet = false;
    iq_correction = 0;
    m_iq_correction_isSet = false;
    hw_bbdc_block = 0;
    m_hw_bbdc_block_isSet = false;
    hw_rfdc_block = 0;
    m_hw_rfdc_block_isSet = false;
    hw_iq_correction = 0;
    m_hw_iq_correction_isSet = false;
    log2_decim = 0;
    m_log2_decim_isSet = false;
    lpf_bw = 0;
    m_lpf_bw_isSet = false;
    gain = 0;
    m_gain_isSet = false;
    antenna_path = 0;
    m_antenna_path_isSet = false;
    gain_mode = 0;
    m_gain_mode_isSet = false;
    transverter_mode = 0;
    m_transverter_mode_isSet = false;
    transverter_delta_frequency = 0L;
    m_transverter_delta_frequency_isSet = false;
    iq_order = 0;
    m_iq_order_isSet = false;
    use_reverse_api = 0;
    m_use_reverse_api_isSet = false;
    reverse_api_address = new QString("");
    m_reverse_api_address_isSet = false;
    reverse_api_port = 0;
    m_reverse_api_port_isSet = false;
    reverse_api_device_index = 0;
    m_reverse_api_device_index_isSet = false;
}

void
SWGZcuadrv9009InputSettings::cleanup() {






















    if(reverse_api_address != nullptr) { 
        delete reverse_api_address;
    }


}

SWGZcuadrv9009InputSettings*
SWGZcuadrv9009InputSettings::fromJson(QString &json) {
    QByteArray array (json.toStdString().c_str());
    QJsonDocument doc = QJsonDocument::fromJson(array);
    QJsonObject jsonObject = doc.object();
    this->fromJsonObject(jsonObject);
    return this;
}

void
SWGZcuadrv9009InputSettings::fromJsonObject(QJsonObject &pJson) {
    ::SWGSDRangel::setValue(&center_frequency, pJson["centerFrequency"], "qint64", "");
    
    ::SWGSDRangel::setValue(&dev_sample_rate, pJson["devSampleRate"], "qint32", "");
    
    ::SWGSDRangel::setValue(&l_oppm_tenths, pJson["LOppmTenths"], "qint32", "");
    
    ::SWGSDRangel::setValue(&lpf_fir_enable, pJson["lpfFIREnable"], "qint32", "");
    
    ::SWGSDRangel::setValue(&lpf_firbw, pJson["lpfFIRBW"], "qint32", "");
    
    ::SWGSDRangel::setValue(&lpf_fi_rlog2_decim, pJson["lpfFIRlog2Decim"], "qint32", "");
    
    ::SWGSDRangel::setValue(&lpf_fir_gain, pJson["lpfFIRGain"], "qint32", "");
    
    ::SWGSDRangel::setValue(&fc_pos, pJson["fcPos"], "qint32", "");
    
    ::SWGSDRangel::setValue(&dc_block, pJson["dcBlock"], "qint32", "");
    
    ::SWGSDRangel::setValue(&iq_correction, pJson["iqCorrection"], "qint32", "");
    
    ::SWGSDRangel::setValue(&hw_bbdc_block, pJson["hwBBDCBlock"], "qint32", "");
    
    ::SWGSDRangel::setValue(&hw_rfdc_block, pJson["hwRFDCBlock"], "qint32", "");
    
    ::SWGSDRangel::setValue(&hw_iq_correction, pJson["hwIQCorrection"], "qint32", "");
    
    ::SWGSDRangel::setValue(&log2_decim, pJson["log2Decim"], "qint32", "");
    
    ::SWGSDRangel::setValue(&lpf_bw, pJson["lpfBW"], "qint32", "");
    
    ::SWGSDRangel::setValue(&gain, pJson["gain"], "qint32", "");
    
    ::SWGSDRangel::setValue(&antenna_path, pJson["antennaPath"], "qint32", "");
    
    ::SWGSDRangel::setValue(&gain_mode, pJson["gainMode"], "qint32", "");
    
    ::SWGSDRangel::setValue(&transverter_mode, pJson["transverterMode"], "qint32", "");
    
    ::SWGSDRangel::setValue(&transverter_delta_frequency, pJson["transverterDeltaFrequency"], "qint64", "");
    
    ::SWGSDRangel::setValue(&iq_order, pJson["iqOrder"], "qint32", "");
    
    ::SWGSDRangel::setValue(&use_reverse_api, pJson["useReverseAPI"], "qint32", "");
    
    ::SWGSDRangel::setValue(&reverse_api_address, pJson["reverseAPIAddress"], "QString", "QString");
    
    ::SWGSDRangel::setValue(&reverse_api_port, pJson["reverseAPIPort"], "qint32", "");
    
    ::SWGSDRangel::setValue(&reverse_api_device_index, pJson["reverseAPIDeviceIndex"], "qint32", "");
    
}

QString
SWGZcuadrv9009InputSettings::asJson ()
{
    QJsonObject* obj = this->asJsonObject();

    QJsonDocument doc(*obj);
    QByteArray bytes = doc.toJson();
    delete obj;
    return QString(bytes);
}

QJsonObject*
SWGZcuadrv9009InputSettings::asJsonObject() {
    QJsonObject* obj = new QJsonObject();
    if(m_center_frequency_isSet){
        obj->insert("centerFrequency", QJsonValue(center_frequency));
    }
    if(m_dev_sample_rate_isSet){
        obj->insert("devSampleRate", QJsonValue(dev_sample_rate));
    }
    if(m_l_oppm_tenths_isSet){
        obj->insert("LOppmTenths", QJsonValue(l_oppm_tenths));
    }
    if(m_lpf_fir_enable_isSet){
        obj->insert("lpfFIREnable", QJsonValue(lpf_fir_enable));
    }
    if(m_lpf_firbw_isSet){
        obj->insert("lpfFIRBW", QJsonValue(lpf_firbw));
    }
    if(m_lpf_fi_rlog2_decim_isSet){
        obj->insert("lpfFIRlog2Decim", QJsonValue(lpf_fi_rlog2_decim));
    }
    if(m_lpf_fir_gain_isSet){
        obj->insert("lpfFIRGain", QJsonValue(lpf_fir_gain));
    }
    if(m_fc_pos_isSet){
        obj->insert("fcPos", QJsonValue(fc_pos));
    }
    if(m_dc_block_isSet){
        obj->insert("dcBlock", QJsonValue(dc_block));
    }
    if(m_iq_correction_isSet){
        obj->insert("iqCorrection", QJsonValue(iq_correction));
    }
    if(m_hw_bbdc_block_isSet){
        obj->insert("hwBBDCBlock", QJsonValue(hw_bbdc_block));
    }
    if(m_hw_rfdc_block_isSet){
        obj->insert("hwRFDCBlock", QJsonValue(hw_rfdc_block));
    }
    if(m_hw_iq_correction_isSet){
        obj->insert("hwIQCorrection", QJsonValue(hw_iq_correction));
    }
    if(m_log2_decim_isSet){
        obj->insert("log2Decim", QJsonValue(log2_decim));
    }
    if(m_lpf_bw_isSet){
        obj->insert("lpfBW", QJsonValue(lpf_bw));
    }
    if(m_gain_isSet){
        obj->insert("gain", QJsonValue(gain));
    }
    if(m_antenna_path_isSet){
        obj->insert("antennaPath", QJsonValue(antenna_path));
    }
    if(m_gain_mode_isSet){
        obj->insert("gainMode", QJsonValue(gain_mode));
    }
    if(m_transverter_mode_isSet){
        obj->insert("transverterMode", QJsonValue(transverter_mode));
    }
    if(m_transverter_delta_frequency_isSet){
        obj->insert("transverterDeltaFrequency", QJsonValue(transverter_delta_frequency));
    }
    if(m_iq_order_isSet){
        obj->insert("iqOrder", QJsonValue(iq_order));
    }
    if(m_use_reverse_api_isSet){
        obj->insert("useReverseAPI", QJsonValue(use_reverse_api));
    }
    if(reverse_api_address != nullptr && *reverse_api_address != QString("")){
        toJsonValue(QString("reverseAPIAddress"), reverse_api_address, obj, QString("QString"));
    }
    if(m_reverse_api_port_isSet){
        obj->insert("reverseAPIPort", QJsonValue(reverse_api_port));
    }
    if(m_reverse_api_device_index_isSet){
        obj->insert("reverseAPIDeviceIndex", QJsonValue(reverse_api_device_index));
    }

    return obj;
}

qint64
SWGZcuadrv9009InputSettings::getCenterFrequency() {
    return center_frequency;
}
void
SWGZcuadrv9009InputSettings::setCenterFrequency(qint64 center_frequency) {
    this->center_frequency = center_frequency;
    this->m_center_frequency_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getDevSampleRate() {
    return dev_sample_rate;
}
void
SWGZcuadrv9009InputSettings::setDevSampleRate(qint32 dev_sample_rate) {
    this->dev_sample_rate = dev_sample_rate;
    this->m_dev_sample_rate_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getLOppmTenths() {
    return l_oppm_tenths;
}
void
SWGZcuadrv9009InputSettings::setLOppmTenths(qint32 l_oppm_tenths) {
    this->l_oppm_tenths = l_oppm_tenths;
    this->m_l_oppm_tenths_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getLpfFirEnable() {
    return lpf_fir_enable;
}
void
SWGZcuadrv9009InputSettings::setLpfFirEnable(qint32 lpf_fir_enable) {
    this->lpf_fir_enable = lpf_fir_enable;
    this->m_lpf_fir_enable_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getLpfFirbw() {
    return lpf_firbw;
}
void
SWGZcuadrv9009InputSettings::setLpfFirbw(qint32 lpf_firbw) {
    this->lpf_firbw = lpf_firbw;
    this->m_lpf_firbw_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getLpfFiRlog2Decim() {
    return lpf_fi_rlog2_decim;
}
void
SWGZcuadrv9009InputSettings::setLpfFiRlog2Decim(qint32 lpf_fi_rlog2_decim) {
    this->lpf_fi_rlog2_decim = lpf_fi_rlog2_decim;
    this->m_lpf_fi_rlog2_decim_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getLpfFirGain() {
    return lpf_fir_gain;
}
void
SWGZcuadrv9009InputSettings::setLpfFirGain(qint32 lpf_fir_gain) {
    this->lpf_fir_gain = lpf_fir_gain;
    this->m_lpf_fir_gain_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getFcPos() {
    return fc_pos;
}
void
SWGZcuadrv9009InputSettings::setFcPos(qint32 fc_pos) {
    this->fc_pos = fc_pos;
    this->m_fc_pos_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getDcBlock() {
    return dc_block;
}
void
SWGZcuadrv9009InputSettings::setDcBlock(qint32 dc_block) {
    this->dc_block = dc_block;
    this->m_dc_block_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getIqCorrection() {
    return iq_correction;
}
void
SWGZcuadrv9009InputSettings::setIqCorrection(qint32 iq_correction) {
    this->iq_correction = iq_correction;
    this->m_iq_correction_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getHwBbdcBlock() {
    return hw_bbdc_block;
}
void
SWGZcuadrv9009InputSettings::setHwBbdcBlock(qint32 hw_bbdc_block) {
    this->hw_bbdc_block = hw_bbdc_block;
    this->m_hw_bbdc_block_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getHwRfdcBlock() {
    return hw_rfdc_block;
}
void
SWGZcuadrv9009InputSettings::setHwRfdcBlock(qint32 hw_rfdc_block) {
    this->hw_rfdc_block = hw_rfdc_block;
    this->m_hw_rfdc_block_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getHwIqCorrection() {
    return hw_iq_correction;
}
void
SWGZcuadrv9009InputSettings::setHwIqCorrection(qint32 hw_iq_correction) {
    this->hw_iq_correction = hw_iq_correction;
    this->m_hw_iq_correction_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getLog2Decim() {
    return log2_decim;
}
void
SWGZcuadrv9009InputSettings::setLog2Decim(qint32 log2_decim) {
    this->log2_decim = log2_decim;
    this->m_log2_decim_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getLpfBw() {
    return lpf_bw;
}
void
SWGZcuadrv9009InputSettings::setLpfBw(qint32 lpf_bw) {
    this->lpf_bw = lpf_bw;
    this->m_lpf_bw_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getGain() {
    return gain;
}
void
SWGZcuadrv9009InputSettings::setGain(qint32 gain) {
    this->gain = gain;
    this->m_gain_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getAntennaPath() {
    return antenna_path;
}
void
SWGZcuadrv9009InputSettings::setAntennaPath(qint32 antenna_path) {
    this->antenna_path = antenna_path;
    this->m_antenna_path_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getGainMode() {
    return gain_mode;
}
void
SWGZcuadrv9009InputSettings::setGainMode(qint32 gain_mode) {
    this->gain_mode = gain_mode;
    this->m_gain_mode_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getTransverterMode() {
    return transverter_mode;
}
void
SWGZcuadrv9009InputSettings::setTransverterMode(qint32 transverter_mode) {
    this->transverter_mode = transverter_mode;
    this->m_transverter_mode_isSet = true;
}

qint64
SWGZcuadrv9009InputSettings::getTransverterDeltaFrequency() {
    return transverter_delta_frequency;
}
void
SWGZcuadrv9009InputSettings::setTransverterDeltaFrequency(qint64 transverter_delta_frequency) {
    this->transverter_delta_frequency = transverter_delta_frequency;
    this->m_transverter_delta_frequency_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getIqOrder() {
    return iq_order;
}
void
SWGZcuadrv9009InputSettings::setIqOrder(qint32 iq_order) {
    this->iq_order = iq_order;
    this->m_iq_order_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getUseReverseApi() {
    return use_reverse_api;
}
void
SWGZcuadrv9009InputSettings::setUseReverseApi(qint32 use_reverse_api) {
    this->use_reverse_api = use_reverse_api;
    this->m_use_reverse_api_isSet = true;
}

QString*
SWGZcuadrv9009InputSettings::getReverseApiAddress() {
    return reverse_api_address;
}
void
SWGZcuadrv9009InputSettings::setReverseApiAddress(QString* reverse_api_address) {
    this->reverse_api_address = reverse_api_address;
    this->m_reverse_api_address_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getReverseApiPort() {
    return reverse_api_port;
}
void
SWGZcuadrv9009InputSettings::setReverseApiPort(qint32 reverse_api_port) {
    this->reverse_api_port = reverse_api_port;
    this->m_reverse_api_port_isSet = true;
}

qint32
SWGZcuadrv9009InputSettings::getReverseApiDeviceIndex() {
    return reverse_api_device_index;
}
void
SWGZcuadrv9009InputSettings::setReverseApiDeviceIndex(qint32 reverse_api_device_index) {
    this->reverse_api_device_index = reverse_api_device_index;
    this->m_reverse_api_device_index_isSet = true;
}


bool
SWGZcuadrv9009InputSettings::isSet(){
    bool isObjectUpdated = false;
    do{
        if(m_center_frequency_isSet){
            isObjectUpdated = true; break;
        }
        if(m_dev_sample_rate_isSet){
            isObjectUpdated = true; break;
        }
        if(m_l_oppm_tenths_isSet){
            isObjectUpdated = true; break;
        }
        if(m_lpf_fir_enable_isSet){
            isObjectUpdated = true; break;
        }
        if(m_lpf_firbw_isSet){
            isObjectUpdated = true; break;
        }
        if(m_lpf_fi_rlog2_decim_isSet){
            isObjectUpdated = true; break;
        }
        if(m_lpf_fir_gain_isSet){
            isObjectUpdated = true; break;
        }
        if(m_fc_pos_isSet){
            isObjectUpdated = true; break;
        }
        if(m_dc_block_isSet){
            isObjectUpdated = true; break;
        }
        if(m_iq_correction_isSet){
            isObjectUpdated = true; break;
        }
        if(m_hw_bbdc_block_isSet){
            isObjectUpdated = true; break;
        }
        if(m_hw_rfdc_block_isSet){
            isObjectUpdated = true; break;
        }
        if(m_hw_iq_correction_isSet){
            isObjectUpdated = true; break;
        }
        if(m_log2_decim_isSet){
            isObjectUpdated = true; break;
        }
        if(m_lpf_bw_isSet){
            isObjectUpdated = true; break;
        }
        if(m_gain_isSet){
            isObjectUpdated = true; break;
        }
        if(m_antenna_path_isSet){
            isObjectUpdated = true; break;
        }
        if(m_gain_mode_isSet){
            isObjectUpdated = true; break;
        }
        if(m_transverter_mode_isSet){
            isObjectUpdated = true; break;
        }
        if(m_transverter_delta_frequency_isSet){
            isObjectUpdated = true; break;
        }
        if(m_iq_order_isSet){
            isObjectUpdated = true; break;
        }
        if(m_use_reverse_api_isSet){
            isObjectUpdated = true; break;
        }
        if(reverse_api_address && *reverse_api_address != QString("")){
            isObjectUpdated = true; break;
        }
        if(m_reverse_api_port_isSet){
            isObjectUpdated = true; break;
        }
        if(m_reverse_api_device_index_isSet){
            isObjectUpdated = true; break;
        }
    }while(false);
    return isObjectUpdated;
}
}

