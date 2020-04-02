/**
 * SDRangel
 * This is the web REST/JSON API of SDRangel SDR software. SDRangel is an Open Source Qt5/OpenGL 3.0+ (4.3+ in Windows) GUI and server Software Defined Radio and signal analyzer in software. It supports Airspy, BladeRF, HackRF, LimeSDR, PlutoSDR, RTL-SDR, SDRplay RSP1 and FunCube    ---   Limitations and specifcities:    * In SDRangel GUI the first Rx device set cannot be deleted. Conversely the server starts with no device sets and its number of device sets can be reduced to zero by as many calls as necessary to /sdrangel/deviceset with DELETE method.   * Preset import and export from/to file is a server only feature.   * Device set focus is a GUI only feature.   * The following channels are not implemented (status 501 is returned): ATV and DATV demodulators, Channel Analyzer NG, LoRa demodulator   * The device settings and report structures contains only the sub-structure corresponding to the device type. The DeviceSettings and DeviceReport structures documented here shows all of them but only one will be or should be present at a time   * The channel settings and report structures contains only the sub-structure corresponding to the channel type. The ChannelSettings and ChannelReport structures documented here shows all of them but only one will be or should be present at a time    --- 
 *
 * OpenAPI spec version: 4.14.0
 * Contact: f4exb06@gmail.com
 *
 * NOTE: This class is auto generated by the swagger code generator program.
 * https://github.com/swagger-api/swagger-codegen.git
 * Do not edit the class manually.
 */

/*
 * SWGDeviceActions.h
 *
 * Base device actions. Only the device actions corresponding to the device specified in the deviceHwType field is or should be present.
 */

#ifndef SWGDeviceActions_H_
#define SWGDeviceActions_H_

#include <QJsonObject>


#include "SWGAirspyActions.h"
#include "SWGAirspyHFActions.h"
#include "SWGBladeRF1InputActions.h"
#include "SWGBladeRF2InputActions.h"
#include "SWGFCDProActions.h"
#include "SWGFCDProPlusActions.h"
#include "SWGHackRFInputActions.h"
#include "SWGKiwiSDRActions.h"
#include "SWGLimeSdrInputActions.h"
#include "SWGLocalInputActions.h"
#include "SWGPerseusActions.h"
#include "SWGPlutoSdrInputActions.h"
#include "SWGRemoteInputActions.h"
#include "SWGRtlSdrActions.h"
#include "SWGSDRPlayActions.h"
#include "SWGSoapySDRInputActions.h"
#include "SWGTestSourceActions.h"
#include "SWGXtrxInputActions.h"
#include <QString>

#include "SWGObject.h"
#include "export.h"

namespace SWGSDRangel {

class SWG_API SWGDeviceActions: public SWGObject {
public:
    SWGDeviceActions();
    SWGDeviceActions(QString* json);
    virtual ~SWGDeviceActions();
    void init();
    void cleanup();

    virtual QString asJson () override;
    virtual QJsonObject* asJsonObject() override;
    virtual void fromJsonObject(QJsonObject &json) override;
    virtual SWGDeviceActions* fromJson(QString &jsonString) override;

    QString* getDeviceHwType();
    void setDeviceHwType(QString* device_hw_type);

    qint32 getDirection();
    void setDirection(qint32 direction);

    qint32 getOriginatorIndex();
    void setOriginatorIndex(qint32 originator_index);

    SWGAirspyActions* getAirspyActions();
    void setAirspyActions(SWGAirspyActions* airspy_actions);

    SWGAirspyHFActions* getAirspyHfActions();
    void setAirspyHfActions(SWGAirspyHFActions* airspy_hf_actions);

    SWGBladeRF1InputActions* getBladeRf1InputActions();
    void setBladeRf1InputActions(SWGBladeRF1InputActions* blade_rf1_input_actions);

    SWGBladeRF2InputActions* getBladeRf2InputActions();
    void setBladeRf2InputActions(SWGBladeRF2InputActions* blade_rf2_input_actions);

    SWGFCDProActions* getFcdProActions();
    void setFcdProActions(SWGFCDProActions* fcd_pro_actions);

    SWGFCDProPlusActions* getFcdProPlusActions();
    void setFcdProPlusActions(SWGFCDProPlusActions* fcd_pro_plus_actions);

    SWGHackRFInputActions* getHackRfInputActions();
    void setHackRfInputActions(SWGHackRFInputActions* hack_rf_input_actions);

    SWGKiwiSDRActions* getKiwiSdrActions();
    void setKiwiSdrActions(SWGKiwiSDRActions* kiwi_sdr_actions);

    SWGLimeSdrInputActions* getLimeSdrInputActions();
    void setLimeSdrInputActions(SWGLimeSdrInputActions* lime_sdr_input_actions);

    SWGLocalInputActions* getLocalInputActions();
    void setLocalInputActions(SWGLocalInputActions* local_input_actions);

    SWGPerseusActions* getPerseusActions();
    void setPerseusActions(SWGPerseusActions* perseus_actions);

    SWGPlutoSdrInputActions* getPlutoSdrInputActions();
    void setPlutoSdrInputActions(SWGPlutoSdrInputActions* pluto_sdr_input_actions);

    SWGRemoteInputActions* getRemoteInputActions();
    void setRemoteInputActions(SWGRemoteInputActions* remote_input_actions);

    SWGRtlSdrActions* getRtlSdrActions();
    void setRtlSdrActions(SWGRtlSdrActions* rtl_sdr_actions);

    SWGSDRPlayActions* getSdrPlayActions();
    void setSdrPlayActions(SWGSDRPlayActions* sdr_play_actions);

    SWGSoapySDRInputActions* getSoapySdrInputActions();
    void setSoapySdrInputActions(SWGSoapySDRInputActions* soapy_sdr_input_actions);

    SWGTestSourceActions* getTestSourceActions();
    void setTestSourceActions(SWGTestSourceActions* test_source_actions);

    SWGXtrxInputActions* getXtrxInputActions();
    void setXtrxInputActions(SWGXtrxInputActions* xtrx_input_actions);


    virtual bool isSet() override;

private:
    QString* device_hw_type;
    bool m_device_hw_type_isSet;

    qint32 direction;
    bool m_direction_isSet;

    qint32 originator_index;
    bool m_originator_index_isSet;

    SWGAirspyActions* airspy_actions;
    bool m_airspy_actions_isSet;

    SWGAirspyHFActions* airspy_hf_actions;
    bool m_airspy_hf_actions_isSet;

    SWGBladeRF1InputActions* blade_rf1_input_actions;
    bool m_blade_rf1_input_actions_isSet;

    SWGBladeRF2InputActions* blade_rf2_input_actions;
    bool m_blade_rf2_input_actions_isSet;

    SWGFCDProActions* fcd_pro_actions;
    bool m_fcd_pro_actions_isSet;

    SWGFCDProPlusActions* fcd_pro_plus_actions;
    bool m_fcd_pro_plus_actions_isSet;

    SWGHackRFInputActions* hack_rf_input_actions;
    bool m_hack_rf_input_actions_isSet;

    SWGKiwiSDRActions* kiwi_sdr_actions;
    bool m_kiwi_sdr_actions_isSet;

    SWGLimeSdrInputActions* lime_sdr_input_actions;
    bool m_lime_sdr_input_actions_isSet;

    SWGLocalInputActions* local_input_actions;
    bool m_local_input_actions_isSet;

    SWGPerseusActions* perseus_actions;
    bool m_perseus_actions_isSet;

    SWGPlutoSdrInputActions* pluto_sdr_input_actions;
    bool m_pluto_sdr_input_actions_isSet;

    SWGRemoteInputActions* remote_input_actions;
    bool m_remote_input_actions_isSet;

    SWGRtlSdrActions* rtl_sdr_actions;
    bool m_rtl_sdr_actions_isSet;

    SWGSDRPlayActions* sdr_play_actions;
    bool m_sdr_play_actions_isSet;

    SWGSoapySDRInputActions* soapy_sdr_input_actions;
    bool m_soapy_sdr_input_actions_isSet;

    SWGTestSourceActions* test_source_actions;
    bool m_test_source_actions_isSet;

    SWGXtrxInputActions* xtrx_input_actions;
    bool m_xtrx_input_actions_isSet;

};

}

#endif /* SWGDeviceActions_H_ */
