///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2017 Edouard Griffiths, F4EXB                                   //
//                                                                               //
// This program is free software; you can redistribute it and/or modify          //
// it under the terms of the GNU General Public License as published by          //
// the Free Software Foundation as version 3 of the License, or                  //
// (at your option) any later version.                                           //
//                                                                               //
// This program is distributed in the hope that it will be useful,               //
// but WITHOUT ANY WARRANTY; without even the implied warranty of                //
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                  //
// GNU General Public License V3 for more details.                               //
//                                                                               //
// You should have received a copy of the GNU General Public License             //
// along with this program. If not, see <http://www.gnu.org/licenses/>.          //
///////////////////////////////////////////////////////////////////////////////////

#include <QDebug>
#include <QNetworkReply>
#include <QBuffer>

#include "SWGDeviceSettings.h"
#include "SWGDeviceState.h"
#include "SWGDeviceReport.h"
#include "SWGZcuadrv9009InputReport.h"

#include "dsp/dspcommands.h"
#include "dsp/dspengine.h"
#include "device/deviceapi.h"
#include "zcuadrv9009/devicezcuadrv9009params.h"
#include "zcuadrv9009/devicezcuadrv9009box.h"

#include "zcuadrv9009input.h"
#include "zcuadrv9009inputthread.h"

#define ZCUADRV9009_BLOCKSIZE_SAMPLES (16*1024) //complex samples per buffer (must be multiple of 64)

MESSAGE_CLASS_DEFINITION(Zcuadrv9009Input::MsgConfigureZcuadrv9009, Message)
MESSAGE_CLASS_DEFINITION(Zcuadrv9009Input::MsgStartStop, Message)

Zcuadrv9009Input::Zcuadrv9009Input(DeviceAPI *deviceAPI) :
    m_deviceAPI(deviceAPI),
    m_deviceDescription("Zcuadrv9009Input"),
    m_running(false),
    m_plutoRxBuffer(0),
    m_zcuadrv9009InputThread(nullptr)
{
    m_sampleFifo.setLabel(m_deviceDescription);
    m_deviceSampleRates.m_addaConnvRate = 0;
    m_deviceSampleRates.m_bbRateHz = 0;
    m_deviceSampleRates.m_firRate = 0;
    m_deviceSampleRates.m_hb1Rate = 0;
    m_deviceSampleRates.m_hb2Rate = 0;
    m_deviceSampleRates.m_hb3Rate = 0;

    suspendBuddies();
    m_open = openDevice();

    if (!m_open) {
        qCritical("Zcuadrv9009Input::Zcuadrv9009Input: cannot open device");
    }

    resumeBuddies();

    m_deviceAPI->setNbSourceStreams(1);

    m_networkManager = new QNetworkAccessManager();
    QObject::connect(
        m_networkManager,
        &QNetworkAccessManager::finished,
        this,
        &Zcuadrv9009Input::networkManagerFinished
    );
}

Zcuadrv9009Input::~Zcuadrv9009Input()
{
    QObject::disconnect(
        m_networkManager,
        &QNetworkAccessManager::finished,
        this,
        &Zcuadrv9009Input::networkManagerFinished
    );
    delete m_networkManager;
    suspendBuddies();
    closeDevice();
    resumeBuddies();
}

void Zcuadrv9009Input::destroy()
{
    delete this;
}

void Zcuadrv9009Input::init()
{
    applySettings(m_settings, QList<QString>(), true);
}

bool Zcuadrv9009Input::start()
{
    if (!m_deviceShared.m_deviceParams->getBox())
    {
        qCritical("Zcuadrv9009Input::start: device not open");
        return false;
    }

    if (m_running) {
        stop();
    }

    // start / stop streaming is done in the thread.

    m_zcuadrv9009InputThread = new Zcuadrv9009InputThread(ZCUADRV9009_BLOCKSIZE_SAMPLES, m_deviceShared.m_deviceParams->getBox(), &m_sampleFifo);
    qDebug("Zcuadrv9009Input::start: thread created");

    applySettings(m_settings, QList<QString>(), true);

    m_zcuadrv9009InputThread->setLog2Decimation(m_settings.m_log2Decim);
    m_zcuadrv9009InputThread->setIQOrder(m_settings.m_iqOrder);
    m_zcuadrv9009InputThread->startWork();

    m_deviceShared.m_thread = m_zcuadrv9009InputThread;
    m_running = true;

    return true;
}

void Zcuadrv9009Input::stop()
{
    if (m_zcuadrv9009InputThread)
    {
        m_zcuadrv9009InputThread->stopWork();
        delete m_zcuadrv9009InputThread;
        m_zcuadrv9009InputThread = nullptr;
    }

    m_deviceShared.m_thread = nullptr;
    m_running = false;
}

QByteArray Zcuadrv9009Input::serialize() const
{
    return m_settings.serialize();
}

bool Zcuadrv9009Input::deserialize(const QByteArray& data)
{
    bool success = true;

    if (!m_settings.deserialize(data))
    {
        m_settings.resetToDefaults();
        success = false;
    }

    MsgConfigureZcuadrv9009* message = MsgConfigureZcuadrv9009::create(m_settings, QList<QString>(), true);
    m_inputMessageQueue.push(message);

    if (m_guiMessageQueue)
    {
        MsgConfigureZcuadrv9009* messageToGUI = MsgConfigureZcuadrv9009::create(m_settings, QList<QString>(), true);
        m_guiMessageQueue->push(messageToGUI);
    }

    return success;
}

const QString& Zcuadrv9009Input::getDeviceDescription() const
{
    return m_deviceDescription;
}
int Zcuadrv9009Input::getSampleRate() const
{
    return (m_settings.m_devSampleRate / (1<<m_settings.m_log2Decim));
}

quint64 Zcuadrv9009Input::getCenterFrequency() const
{
    return m_settings.m_centerFrequency;
}

void Zcuadrv9009Input::setCenterFrequency(qint64 centerFrequency)
{
    Zcuadrv9009InputSettings settings = m_settings;
    settings.m_centerFrequency = centerFrequency;

    MsgConfigureZcuadrv9009* message = MsgConfigureZcuadrv9009::create(settings, QList<QString>{"centerFrequency"}, false);
    m_inputMessageQueue.push(message);

    if (m_guiMessageQueue)
    {
        MsgConfigureZcuadrv9009* messageToGUI = MsgConfigureZcuadrv9009::create(settings, QList<QString>{"centerFrequency"}, false);
        m_guiMessageQueue->push(messageToGUI);
    }
}

bool Zcuadrv9009Input::handleMessage(const Message& message)
{
    if (MsgConfigureZcuadrv9009::match(message))
    {
        MsgConfigureZcuadrv9009& conf = (MsgConfigureZcuadrv9009&) message;
        qDebug() << "Zcuadrv9009Input::handleMessage: MsgConfigureZcuadrv9009";

        if (!applySettings(conf.getSettings(), conf.getSettingsKeys(), conf.getForce())) {
            qDebug("Zcuadrv9009Input::handleMessage config error");
        }

        return true;
    }
    else if (MsgStartStop::match(message))
    {
        MsgStartStop& cmd = (MsgStartStop&) message;
        qDebug() << "Zcuadrv9009Input::handleMessage: MsgStartStop: " << (cmd.getStartStop() ? "start" : "stop");

        if (cmd.getStartStop())
        {
            if (m_deviceAPI->initDeviceEngine())
            {
                m_deviceAPI->startDeviceEngine();
            }
        }
        else
        {
            m_deviceAPI->stopDeviceEngine();
        }

        if (m_settings.m_useReverseAPI) {
            webapiReverseSendStartStop(cmd.getStartStop());
        }

        return true;
    }
    else if (DeviceZcuadrv9009Shared::MsgCrossReportToBuddy::match(message)) // message from buddy
    {
        DeviceZcuadrv9009Shared::MsgCrossReportToBuddy& conf = (DeviceZcuadrv9009Shared::MsgCrossReportToBuddy&) message;
        m_settings.m_devSampleRate = conf.getDevSampleRate();
        m_settings.m_lpfFIRlog2Decim = conf.getLpfFiRlog2IntDec();
        m_settings.m_lpfFIRBW = conf.getLpfFirbw();
        m_settings.m_LOppmTenths = conf.getLoPPMTenths();
        Zcuadrv9009InputSettings newSettings = m_settings;
        newSettings.m_lpfFIREnable = conf.isLpfFirEnable();
        applySettings(newSettings, QList<QString>{"devSampleRate", "lpfFIRlog2Decim", "lpfFIRBW", "LOppmTenths", "lpfFIREnable"});

        return true;
    }
    else
    {
        return false;
    }
}

bool Zcuadrv9009Input::openDevice()
{
    if (!m_sampleFifo.setSize(ZCUADRV9009_BLOCKSIZE_SAMPLES))
    {
        qCritical("Zcuadrv9009Input::openDevice: could not allocate SampleFifo");
        return false;
    }
    else
    {
        qDebug("Zcuadrv9009Input::openDevice: allocated SampleFifo");
    }

    // look for Tx buddy and get reference to common parameters
    if (m_deviceAPI->getSinkBuddies().size() > 0) // then sink
    {
        qDebug("Zcuadrv9009Input::openDevice: look at Tx buddy");

        DeviceAPI *sinkBuddy = m_deviceAPI->getSinkBuddies()[0];
        DeviceZcuadrv9009Shared* buddySharedPtr = (DeviceZcuadrv9009Shared*) sinkBuddy->getBuddySharedPtr();
        m_deviceShared.m_deviceParams = buddySharedPtr->m_deviceParams;

        if (m_deviceShared.m_deviceParams == 0)
        {
            qCritical("Zcuadrv9009Input::openDevice: cannot get device parameters from Tx buddy");
            return false; // the device params should have been created by the buddy
        }
        else
        {
            qDebug("Zcuadrv9009Input::openDevice: getting device parameters from Tx buddy");
        }
    }
    // There is no buddy then create the first Zcuadrv9009 common parameters
    // open the device this will also populate common fields
    else
    {
        qDebug("Zcuadrv9009Input::openDevice: open device here");
        m_deviceShared.m_deviceParams = new DeviceZcuadrv9009Params();

        if (m_deviceAPI->getHardwareUserArguments().size() != 0)
        {
            QStringList kv = m_deviceAPI->getHardwareUserArguments().split('='); // expecting "uri=xxx"

            if (kv.size() > 1)
            {
                if (kv.at(0) == "uri")
                {
                    if (!m_deviceShared.m_deviceParams->openURI(kv.at(1).toStdString()))
                    {
                        qCritical("Zcuadrv9009Input::openDevice: open network device uri=%s failed", qPrintable(kv.at(1)));
                        return false;
                    }
                }
                else
                {
                    qCritical("Zcuadrv9009Input::openDevice: unexpected user parameter key %s", qPrintable(kv.at(0)));
                    return false;
                }
            }
            else
            {
                qCritical("Zcuadrv9009Input::openDevice: unexpected user arguments %s", qPrintable(m_deviceAPI->getHardwareUserArguments()));
                return false;
            }
        }
        else
        {
            char serial[256];
            strcpy(serial, qPrintable(m_deviceAPI->getSamplingDeviceSerial()));

            if (!m_deviceShared.m_deviceParams->open(serial))
            {
                qCritical("Zcuadrv9009Input::openDevice: open serial %s failed", serial);
                return false;
            }
        }
    }

    m_deviceAPI->setBuddySharedPtr(&m_deviceShared); // propagate common parameters to API

    // acquire the channel
    DeviceZcuadrv9009Box *plutoBox =  m_deviceShared.m_deviceParams->getBox();

    if (!plutoBox->openRx())
    {
        qCritical("Zcuadrv9009Input::openDevice: cannot open Rx channel");
        return false;
    }

    m_plutoRxBuffer = plutoBox->createRxBuffer(ZCUADRV9009_BLOCKSIZE_SAMPLES, false);

    return true;
}

void Zcuadrv9009Input::closeDevice()
{
    if (!m_open) { // was never open
        return;
    }

    if (m_deviceAPI->getSinkBuddies().size() == 0)
    {
        m_deviceShared.m_deviceParams->close();
        delete m_deviceShared.m_deviceParams;
        m_deviceShared.m_deviceParams = 0;
    }
}

void Zcuadrv9009Input::suspendBuddies()
{
    // suspend Tx buddy's thread

    for (unsigned int i = 0; i < m_deviceAPI->getSinkBuddies().size(); i++)
    {
        DeviceAPI *buddy = m_deviceAPI->getSinkBuddies()[i];
        DeviceZcuadrv9009Shared *buddyShared = (DeviceZcuadrv9009Shared *) buddy->getBuddySharedPtr();

        if (buddyShared->m_thread) {
            buddyShared->m_thread->stopWork();
        }
    }
}

void Zcuadrv9009Input::resumeBuddies()
{
    // resume Tx buddy's thread

    for (unsigned int i = 0; i < m_deviceAPI->getSinkBuddies().size(); i++)
    {
        DeviceAPI *buddy = m_deviceAPI->getSinkBuddies()[i];
        DeviceZcuadrv9009Shared *buddyShared = (DeviceZcuadrv9009Shared *) buddy->getBuddySharedPtr();

        if (buddyShared->m_thread) {
            buddyShared->m_thread->startWork();
        }
    }
}

bool Zcuadrv9009Input::applySettings(const Zcuadrv9009InputSettings& settings, const QList<QString>& settingsKeys, bool force)
{
    if (!m_open)
    {
        qCritical("Zcuadrv9009Input::applySettings: device not open");
        return false;
    }

    bool forwardChangeOwnDSP    = false;
    bool forwardChangeOtherDSP  = false;
    bool ownThreadWasRunning    = false;
    bool suspendAllOtherThreads = false; // All others means Tx in fact
    DeviceZcuadrv9009Box *plutoBox =  m_deviceShared.m_deviceParams->getBox();
    QLocale loc;
    QList<QString> reverseAPIKeys;

    qDebug() << "Zcuadrv9009Input::applySettings: force: " << force << settings.getDebugString(settingsKeys, force);

    // determine if buddies threads or own thread need to be suspended

    // changes affecting all buddies can occur if
    //   - device to host sample rate is changed
    //   - FIR filter is enabled or disabled
    //   - FIR filter is changed
    //   - LO correction is changed
    if (settingsKeys.contains("devSampleRate") ||
        settingsKeys.contains("lpfFIREnable") ||
        settingsKeys.contains("lpfFIRlog2Decim") ||
        settingsKeys.contains("lpfFIRBW") ||
        settingsKeys.contains("lpfFIRGain") ||
        settingsKeys.contains("LOppmTenths") || force)
    {
        suspendAllOtherThreads = true;
    }

    if (suspendAllOtherThreads)
    {
        const std::vector<DeviceAPI*>& sinkBuddies = m_deviceAPI->getSinkBuddies();
        std::vector<DeviceAPI*>::const_iterator itSink = sinkBuddies.begin();

        for (; itSink != sinkBuddies.end(); ++itSink)
        {
            DeviceZcuadrv9009Shared *buddySharedPtr = (DeviceZcuadrv9009Shared *) (*itSink)->getBuddySharedPtr();

            if (buddySharedPtr->m_thread) {
                buddySharedPtr->m_thread->stopWork();
                buddySharedPtr->m_threadWasRunning = true;
            }
            else
            {
                buddySharedPtr->m_threadWasRunning = false;
            }
        }
    }

    if (m_zcuadrv9009InputThread && m_zcuadrv9009InputThread->isRunning())
    {
        m_zcuadrv9009InputThread->stopWork();
        ownThreadWasRunning = true;
    }

    // apply settings

    if (settingsKeys.contains("dcBlock") ||
        settingsKeys.contains("iqCorrection") || force)
    {
        m_deviceAPI->configureCorrections(settings.m_dcBlock, m_settings.m_iqCorrection);
    }

    // Change affecting device sample rate chain and other buddies
    if (settingsKeys.contains("devSampleRate") ||
        settingsKeys.contains("lpfFIREnable") ||
        settingsKeys.contains("lpfFIRlog2Decim") ||
        settingsKeys.contains("lpfFIRBW") ||
        settingsKeys.contains("lpfFIRGain") || force)
    {
        plutoBox->setFIR(settings.m_devSampleRate, settings.m_lpfFIRlog2Decim, DeviceZcuadrv9009Box::USE_RX, settings.m_lpfFIRBW, settings.m_lpfFIRGain);
        plutoBox->setFIREnable(settings.m_lpfFIREnable);   // eventually enable/disable FIR
        plutoBox->setSampleRate(settings.m_devSampleRate); // and set end point sample rate

        plutoBox->getRxSampleRates(m_deviceSampleRates); // pick up possible new rates
        qDebug() << "Zcuadrv9009Input::applySettings: BBPLL(Hz): " << m_deviceSampleRates.m_bbRateHz
                 << " ADC: " << m_deviceSampleRates.m_addaConnvRate
                 << " -HB3-> " << m_deviceSampleRates.m_hb3Rate
                 << " -HB2-> " << m_deviceSampleRates.m_hb2Rate
                 << " -HB1-> " << m_deviceSampleRates.m_hb1Rate
                 << " -FIR-> " << m_deviceSampleRates.m_firRate;

        forwardChangeOtherDSP = true;
        forwardChangeOwnDSP = (m_settings.m_devSampleRate != settings.m_devSampleRate) || force;
    }

    if (settingsKeys.contains("log2Decim") || force)
    {
        if (m_zcuadrv9009InputThread)
        {
            m_zcuadrv9009InputThread->setLog2Decimation(settings.m_log2Decim);
            qDebug() << "Zcuadrv9009Input::applySettings: set soft decimation to " << (1<<settings.m_log2Decim);
        }

        forwardChangeOwnDSP = true;
    }

    if (settingsKeys.contains("iqOrder") || force)
    {
        if (m_zcuadrv9009InputThread) {
            m_zcuadrv9009InputThread->setIQOrder(settings.m_iqOrder);
        }
    }

    if (settingsKeys.contains("LOppmTenths") || force)
    {
        plutoBox->setLOPPMTenths(settings.m_LOppmTenths);
        forwardChangeOtherDSP = true;
    }

    std::vector<std::string> params;
    bool paramsToSet = false;

    if (settingsKeys.contains("centerFrequency")
        || settingsKeys.contains("fcPos")
        || settingsKeys.contains("log2Decim")
        || settingsKeys.contains("devSampleRate")
        || settingsKeys.contains("transverterMode")
        || settingsKeys.contains("transverterDeltaFrequency") || force)
    {
        qint64 deviceCenterFrequency = DeviceSampleSource::calculateDeviceCenterFrequency(
                settings.m_centerFrequency,
                settings.m_transverterDeltaFrequency,
                settings.m_log2Decim,
                (DeviceSampleSource::fcPos_t) settings.m_fcPos,
                settings.m_devSampleRate,
                DeviceSampleSource::FrequencyShiftScheme::FSHIFT_STD,
                settings.m_transverterMode);

        params.push_back(QString(tr("out_altvoltage0_RX_LO_frequency=%1").arg(deviceCenterFrequency)).toStdString());
        paramsToSet = true;
        forwardChangeOwnDSP = true;

        if (settingsKeys.contains("fcPos") || force)
        {
            if (m_zcuadrv9009InputThread)
            {
                m_zcuadrv9009InputThread->setFcPos(settings.m_fcPos);
                qDebug() << "Zcuadrv9009Input::applySettings: set fcPos to " << settings.m_fcPos;
            }
        }
    }

    if (settingsKeys.contains("lpfBW") || force)
    {
        params.push_back(QString(tr("in_voltage_rf_bandwidth=%1").arg(settings.m_lpfBW)).toStdString());
        paramsToSet = true;
    }

    if (settingsKeys.contains("antennaPath") || force)
    {
        QString rfPortStr;
        Zcuadrv9009InputSettings::translateRFPath(settings.m_antennaPath, rfPortStr);
        params.push_back(QString(tr("in_voltage0_rf_port_select=%1").arg(rfPortStr)).toStdString());
        paramsToSet = true;
    }

    if (settingsKeys.contains("gainMode") || force)
    {
        QString gainModeStr;
        Zcuadrv9009InputSettings::translateGainMode(settings.m_gainMode, gainModeStr);
        params.push_back(QString(tr("in_voltage0_gain_control_mode=%1").arg(gainModeStr)).toStdString());
        paramsToSet = true;
    }

    if (settingsKeys.contains("gain") || force)
    {
        params.push_back(QString(tr("in_voltage0_hardwaregain=%1").arg(settings.m_gain)).toStdString());
        paramsToSet = true;
    }

    if (settingsKeys.contains("hwBBDCBlock") || force)
    {
        params.push_back(QString(tr("in_voltage_bb_dc_offset_tracking_en=%1").arg(settings.m_hwBBDCBlock ? 1 : 0)).toStdString());
        paramsToSet = true;
    }

    if (settingsKeys.contains("hwRFDCBlock") || force)
    {
        params.push_back(QString(tr("in_voltage_rf_dc_offset_tracking_en=%1").arg(settings.m_hwRFDCBlock ? 1 : 0)).toStdString());
        paramsToSet = true;
    }

    if (settingsKeys.contains("hwIQCorrection") || force)
    {
        params.push_back(QString(tr("in_voltage_quadrature_tracking_en=%1").arg(settings.m_hwIQCorrection ? 1 : 0)).toStdString());
        paramsToSet = true;
    }

    if (paramsToSet)
    {
        plutoBox->set_params(DeviceZcuadrv9009Box::DEVICE_PHY, params);
    }

    if (settingsKeys.contains("useReverseAPI"))
    {
        bool fullUpdate = (settingsKeys.contains("useReverseAPI") && settings.m_useReverseAPI) ||
            settingsKeys.contains("reverseAPIAddress") ||
            settingsKeys.contains("reverseAPIPort") ||
            settingsKeys.contains("reverseAPIDeviceIndex");
        webapiReverseSendSettings(settingsKeys, settings, fullUpdate || force);
    }

    if (force) {
        m_settings = settings;
    } else {
        m_settings.applySettings(settingsKeys, settings);
    }

    if (suspendAllOtherThreads)
    {
        const std::vector<DeviceAPI*>& sinkBuddies = m_deviceAPI->getSinkBuddies();
        std::vector<DeviceAPI*>::const_iterator itSink = sinkBuddies.begin();

        for (; itSink != sinkBuddies.end(); ++itSink)
        {
            DeviceZcuadrv9009Shared *buddySharedPtr = (DeviceZcuadrv9009Shared *) (*itSink)->getBuddySharedPtr();

            if (buddySharedPtr->m_threadWasRunning) {
                buddySharedPtr->m_thread->startWork();
            }
        }
    }

    if (ownThreadWasRunning) {
        m_zcuadrv9009InputThread->startWork();
    }

    // TODO: forward changes to other (Tx) DSP
    if (forwardChangeOtherDSP)
    {

        qDebug("Zcuadrv9009Input::applySettings: forwardChangeOtherDSP");

        const std::vector<DeviceAPI*>& sinkBuddies = m_deviceAPI->getSinkBuddies();
        std::vector<DeviceAPI*>::const_iterator itSink = sinkBuddies.begin();

        for (; itSink != sinkBuddies.end(); ++itSink)
        {
            DeviceZcuadrv9009Shared::MsgCrossReportToBuddy *msg = DeviceZcuadrv9009Shared::MsgCrossReportToBuddy::create(
                    settings.m_devSampleRate,
                    settings.m_lpfFIREnable,
                    settings.m_lpfFIRlog2Decim,
                    settings.m_lpfFIRBW,
                    settings.m_LOppmTenths);

            if ((*itSink)->getSamplingDeviceGUIMessageQueue())
            {
                DeviceZcuadrv9009Shared::MsgCrossReportToBuddy *msgToGUI = new DeviceZcuadrv9009Shared::MsgCrossReportToBuddy(*msg);
                (*itSink)->getSamplingDeviceGUIMessageQueue()->push(msgToGUI);
            }

            (*itSink)->getSamplingDeviceInputMessageQueue()->push(msg);
        }
    }

    if (forwardChangeOwnDSP)
    {
        qDebug("Zcuadrv9009Input::applySettings: forward change to self");

        int sampleRate = m_settings.m_devSampleRate/(1<<m_settings.m_log2Decim);
        DSPSignalNotification *notif = new DSPSignalNotification(sampleRate, m_settings.m_centerFrequency);
        m_deviceAPI->getDeviceEngineInputMessageQueue()->push(notif);
    }

    return true;
}

void Zcuadrv9009Input::getRSSI(std::string& rssiStr)
{
    if (!m_open)
    {
        qDebug("Zcuadrv9009Input::getRSSI: device not open");
        return;
    }

    DeviceZcuadrv9009Box *plutoBox =  m_deviceShared.m_deviceParams->getBox();

    if (!plutoBox->getRxRSSI(rssiStr, 0)) {
        rssiStr = "xxx dB";
    }
}

void Zcuadrv9009Input::getLORange(qint64& minLimit, qint64& maxLimit)
{
    if (!m_open)
    {
        qDebug("Zcuadrv9009Input::getLORange: device not open");
        return;
    }

    uint64_t min, max;
    DeviceZcuadrv9009Box *plutoBox = m_deviceShared.m_deviceParams->getBox();

    plutoBox->getRxLORange(min, max);
    minLimit = min;
    maxLimit = max;
}

void Zcuadrv9009Input::getbbLPRange(quint32& minLimit, quint32& maxLimit)
{
    if (!m_open)
    {
        qDebug("Zcuadrv9009Input::getbbLPRange: device not open");
        return;
    }

    uint32_t min, max;
    DeviceZcuadrv9009Box *plutoBox =  m_deviceShared.m_deviceParams->getBox();

    plutoBox->getbbLPRxRange(min, max);
    minLimit = min;
    maxLimit = max;
}

void Zcuadrv9009Input::getGain(int& gaindB)
{
    if (!m_open)
    {
        qDebug("Zcuadrv9009Input::getGain: device not open");
        return;
    }

    DeviceZcuadrv9009Box *plutoBox =  m_deviceShared.m_deviceParams->getBox();

    if (!plutoBox->getRxGain(gaindB, 0)) {
        gaindB = 0;
    }
}

bool Zcuadrv9009Input::fetchTemperature()
{
    if (!m_open)
    {
        qDebug("Zcuadrv9009Input::fetchTemperature: device not open");
        return false;
    }

    DeviceZcuadrv9009Box *plutoBox =  m_deviceShared.m_deviceParams->getBox();
    return plutoBox->fetchTemp();
}

float Zcuadrv9009Input::getTemperature()
{
    if (!m_open)
    {
        qDebug("Zcuadrv9009Input::getTemperature: device not open");
        return 0.0;
    }

    DeviceZcuadrv9009Box *plutoBox =  m_deviceShared.m_deviceParams->getBox();
    return plutoBox->getTemp();
}

int Zcuadrv9009Input::webapiRunGet(
        SWGSDRangel::SWGDeviceState& response,
        QString& errorMessage)
{
    (void) errorMessage;
    m_deviceAPI->getDeviceEngineStateStr(*response.getState());
    return 200;
}

int Zcuadrv9009Input::webapiRun(
        bool run,
        SWGSDRangel::SWGDeviceState& response,
        QString& errorMessage)
{
    (void) errorMessage;
    m_deviceAPI->getDeviceEngineStateStr(*response.getState());
    MsgStartStop *message = MsgStartStop::create(run);
    m_inputMessageQueue.push(message);

    if (m_guiMessageQueue) // forward to GUI if any
    {
        MsgStartStop *msgToGUI = MsgStartStop::create(run);
        m_guiMessageQueue->push(msgToGUI);
    }

    return 200;
}

int Zcuadrv9009Input::webapiSettingsGet(
                SWGSDRangel::SWGDeviceSettings& response,
                QString& errorMessage)
{
    (void) errorMessage;
    response.setZcuadrv9009InputSettings(new SWGSDRangel::SWGZcuadrv9009InputSettings());
    response.getZcuadrv9009InputSettings()->init();
    webapiFormatDeviceSettings(response, m_settings);
    return 200;
}

int Zcuadrv9009Input::webapiSettingsPutPatch(
                bool force,
                const QStringList& deviceSettingsKeys,
                SWGSDRangel::SWGDeviceSettings& response, // query + response
                QString& errorMessage)
{
    (void) errorMessage;
    Zcuadrv9009InputSettings settings = m_settings;
    webapiUpdateDeviceSettings(settings, deviceSettingsKeys, response);

    MsgConfigureZcuadrv9009 *msg = MsgConfigureZcuadrv9009::create(settings, deviceSettingsKeys, force);
    m_inputMessageQueue.push(msg);

    if (m_guiMessageQueue) // forward to GUI if any
    {
        MsgConfigureZcuadrv9009 *msgToGUI = MsgConfigureZcuadrv9009::create(settings, deviceSettingsKeys, force);
        m_guiMessageQueue->push(msgToGUI);
    }

    webapiFormatDeviceSettings(response, settings);
    return 200;
}

void Zcuadrv9009Input::webapiUpdateDeviceSettings(
        Zcuadrv9009InputSettings& settings,
        const QStringList& deviceSettingsKeys,
        SWGSDRangel::SWGDeviceSettings& response)
{
    if (deviceSettingsKeys.contains("centerFrequency")) {
        settings.m_centerFrequency = response.getZcuadrv9009InputSettings()->getCenterFrequency();
    }
    if (deviceSettingsKeys.contains("devSampleRate")) {
        settings.m_devSampleRate = response.getZcuadrv9009InputSettings()->getDevSampleRate();
    }
    if (deviceSettingsKeys.contains("LOppmTenths")) {
        settings.m_LOppmTenths = response.getZcuadrv9009InputSettings()->getLOppmTenths();
    }
    if (deviceSettingsKeys.contains("lpfFIREnable")) {
        settings.m_lpfFIREnable = response.getZcuadrv9009InputSettings()->getLpfFirEnable() != 0;
    }
    if (deviceSettingsKeys.contains("lpfFIRBW")) {
        settings.m_lpfFIRBW = response.getZcuadrv9009InputSettings()->getLpfFirbw();
    }
    if (deviceSettingsKeys.contains("lpfFIRlog2Decim")) {
        settings.m_lpfFIRlog2Decim = response.getZcuadrv9009InputSettings()->getLpfFiRlog2Decim();
    }
    if (deviceSettingsKeys.contains("lpfFIRGain")) {
        settings.m_lpfFIRGain = response.getZcuadrv9009InputSettings()->getLpfFirGain();
    }
    if (deviceSettingsKeys.contains("fcPos")) {
        int fcPos = response.getZcuadrv9009InputSettings()->getFcPos();
        fcPos = fcPos < 0 ? 0 : fcPos > 2 ? 2 : fcPos;
        settings.m_fcPos = (Zcuadrv9009InputSettings::fcPos_t) fcPos;
    }
    if (deviceSettingsKeys.contains("dcBlock")) {
        settings.m_dcBlock = response.getZcuadrv9009InputSettings()->getDcBlock() != 0;
    }
    if (deviceSettingsKeys.contains("iqCorrection")) {
        settings.m_iqCorrection = response.getZcuadrv9009InputSettings()->getIqCorrection() != 0;
    }
    if (deviceSettingsKeys.contains("hwBBDCBlock")) {
        settings.m_hwBBDCBlock = response.getZcuadrv9009InputSettings()->getHwBbdcBlock() != 0;
    }
    if (deviceSettingsKeys.contains("hwRFDCBlock")) {
        settings.m_hwBBDCBlock = response.getZcuadrv9009InputSettings()->getHwRfdcBlock() != 0;
    }
    if (deviceSettingsKeys.contains("hwIQCorrection")) {
        settings.m_hwBBDCBlock = response.getZcuadrv9009InputSettings()->getHwIqCorrection() != 0;
    }
    if (deviceSettingsKeys.contains("log2Decim")) {
        settings.m_log2Decim = response.getZcuadrv9009InputSettings()->getLog2Decim();
    }
    if (deviceSettingsKeys.contains("iqOrder")) {
        settings.m_iqOrder = response.getZcuadrv9009InputSettings()->getIqOrder() != 0;
    }
    if (deviceSettingsKeys.contains("lpfBW")) {
        settings.m_lpfBW = response.getZcuadrv9009InputSettings()->getLpfBw();
    }
    if (deviceSettingsKeys.contains("gain")) {
        settings.m_gain = response.getZcuadrv9009InputSettings()->getGain();
    }
    if (deviceSettingsKeys.contains("antennaPath")) {
        int antennaPath = response.getZcuadrv9009InputSettings()->getAntennaPath();
        antennaPath = antennaPath < 0 ? 0 : antennaPath >= Zcuadrv9009InputSettings::RFPATH_END ? Zcuadrv9009InputSettings::RFPATH_END-1 : antennaPath;
        settings.m_antennaPath = (Zcuadrv9009InputSettings::RFPath) antennaPath;
    }
    if (deviceSettingsKeys.contains("gainMode")) {
        int gainMode = response.getZcuadrv9009InputSettings()->getGainMode();
        gainMode = gainMode < 0 ? 0 : gainMode >= Zcuadrv9009InputSettings::GAIN_END ? Zcuadrv9009InputSettings::GAIN_END-1 : gainMode;
        settings.m_gainMode = (Zcuadrv9009InputSettings::GainMode) gainMode;
    }
    if (deviceSettingsKeys.contains("transverterDeltaFrequency")) {
        settings.m_transverterDeltaFrequency = response.getZcuadrv9009InputSettings()->getTransverterDeltaFrequency();
    }
    if (deviceSettingsKeys.contains("transverterMode")) {
        settings.m_transverterMode = response.getZcuadrv9009InputSettings()->getTransverterMode() != 0;
    }
    if (deviceSettingsKeys.contains("useReverseAPI")) {
        settings.m_useReverseAPI = response.getZcuadrv9009InputSettings()->getUseReverseApi() != 0;
    }
    if (deviceSettingsKeys.contains("reverseAPIAddress")) {
        settings.m_reverseAPIAddress = *response.getZcuadrv9009InputSettings()->getReverseApiAddress();
    }
    if (deviceSettingsKeys.contains("reverseAPIPort")) {
        settings.m_reverseAPIPort = response.getZcuadrv9009InputSettings()->getReverseApiPort();
    }
    if (deviceSettingsKeys.contains("reverseAPIDeviceIndex")) {
        settings.m_reverseAPIDeviceIndex = response.getZcuadrv9009InputSettings()->getReverseApiDeviceIndex();
    }
}

int Zcuadrv9009Input::webapiReportGet(
        SWGSDRangel::SWGDeviceReport& response,
        QString& errorMessage)
{
    (void) errorMessage;
    response.setZcuadrv9009InputReport(new SWGSDRangel::SWGZcuadrv9009InputReport());
    response.getZcuadrv9009InputReport()->init();
    webapiFormatDeviceReport(response);
    return 200;
}

void Zcuadrv9009Input::webapiFormatDeviceSettings(SWGSDRangel::SWGDeviceSettings& response, const Zcuadrv9009InputSettings& settings)
{
    response.getZcuadrv9009InputSettings()->setCenterFrequency(settings.m_centerFrequency);
    response.getZcuadrv9009InputSettings()->setDevSampleRate(settings.m_devSampleRate);
    response.getZcuadrv9009InputSettings()->setLOppmTenths(settings.m_LOppmTenths);
    response.getZcuadrv9009InputSettings()->setLpfFirEnable(settings.m_lpfFIREnable ? 1 : 0);
    response.getZcuadrv9009InputSettings()->setLpfFirbw(settings.m_lpfFIRBW);
    response.getZcuadrv9009InputSettings()->setLpfFiRlog2Decim(settings.m_lpfFIRlog2Decim);
    response.getZcuadrv9009InputSettings()->setLpfFirGain(settings.m_lpfFIRGain);
    response.getZcuadrv9009InputSettings()->setFcPos((int) settings.m_fcPos);
    response.getZcuadrv9009InputSettings()->setDcBlock(settings.m_dcBlock ? 1 : 0);
    response.getZcuadrv9009InputSettings()->setIqCorrection(settings.m_iqCorrection ? 1 : 0);
    response.getZcuadrv9009InputSettings()->setHwBbdcBlock(settings.m_hwBBDCBlock ? 1 : 0);
    response.getZcuadrv9009InputSettings()->setHwRfdcBlock(settings.m_hwRFDCBlock ? 1 : 0);
    response.getZcuadrv9009InputSettings()->setHwIqCorrection(settings.m_hwIQCorrection ? 1 : 0);
    response.getZcuadrv9009InputSettings()->setLog2Decim(settings.m_log2Decim);
    response.getZcuadrv9009InputSettings()->setIqOrder(settings.m_iqOrder ? 1 : 0);
    response.getZcuadrv9009InputSettings()->setLpfBw(settings.m_lpfBW);
    response.getZcuadrv9009InputSettings()->setGain(settings.m_gain);
    response.getZcuadrv9009InputSettings()->setAntennaPath((int) settings.m_antennaPath);
    response.getZcuadrv9009InputSettings()->setGainMode((int) settings.m_gainMode);
    response.getZcuadrv9009InputSettings()->setTransverterDeltaFrequency(settings.m_transverterDeltaFrequency);
    response.getZcuadrv9009InputSettings()->setTransverterMode(settings.m_transverterMode ? 1 : 0);

    response.getZcuadrv9009InputSettings()->setUseReverseApi(settings.m_useReverseAPI ? 1 : 0);

    if (response.getZcuadrv9009InputSettings()->getReverseApiAddress()) {
        *response.getZcuadrv9009InputSettings()->getReverseApiAddress() = settings.m_reverseAPIAddress;
    } else {
        response.getZcuadrv9009InputSettings()->setReverseApiAddress(new QString(settings.m_reverseAPIAddress));
    }

    response.getZcuadrv9009InputSettings()->setReverseApiPort(settings.m_reverseAPIPort);
    response.getZcuadrv9009InputSettings()->setReverseApiDeviceIndex(settings.m_reverseAPIDeviceIndex);
}

void Zcuadrv9009Input::webapiFormatDeviceReport(SWGSDRangel::SWGDeviceReport& response)
{
    response.getZcuadrv9009InputReport()->setAdcRate(getADCSampleRate());
    std::string rssiStr;
    getRSSI(rssiStr);
    response.getZcuadrv9009InputReport()->setRssi(new QString(rssiStr.c_str()));
    int gainDB;
    getGain(gainDB);
    response.getZcuadrv9009InputReport()->setGainDb(gainDB);
    fetchTemperature();
    response.getZcuadrv9009InputReport()->setTemperature(getTemperature());
}

void Zcuadrv9009Input::webapiReverseSendSettings(const QList<QString>& deviceSettingsKeys, const Zcuadrv9009InputSettings& settings, bool force)
{
    SWGSDRangel::SWGDeviceSettings *swgDeviceSettings = new SWGSDRangel::SWGDeviceSettings();
    swgDeviceSettings->setDirection(0); // single Rx
    swgDeviceSettings->setOriginatorIndex(m_deviceAPI->getDeviceSetIndex());
    swgDeviceSettings->setDeviceHwType(new QString("Zcuadrv9009"));
    swgDeviceSettings->setZcuadrv9009InputSettings(new SWGSDRangel::SWGZcuadrv9009InputSettings());
    SWGSDRangel::SWGZcuadrv9009InputSettings *swgZcuadrv9009InputSettings = swgDeviceSettings->getZcuadrv9009InputSettings();

    // transfer data that has been modified. When force is on transfer all data except reverse API data

    if (deviceSettingsKeys.contains("centerFrequency") || force) {
        swgZcuadrv9009InputSettings->setCenterFrequency(settings.m_centerFrequency);
    }
    if (deviceSettingsKeys.contains("devSampleRate") || force) {
        swgZcuadrv9009InputSettings->setDevSampleRate(settings.m_devSampleRate);
    }
    if (deviceSettingsKeys.contains("LOppmTenths") || force) {
        swgZcuadrv9009InputSettings->setLOppmTenths(settings.m_LOppmTenths);
    }
    if (deviceSettingsKeys.contains("lpfFIREnable") || force) {
        swgZcuadrv9009InputSettings->setLpfFirEnable(settings.m_lpfFIREnable ? 1 : 0);
    }
    if (deviceSettingsKeys.contains("lpfFIRBW") || force) {
        swgZcuadrv9009InputSettings->setLpfFirbw(settings.m_lpfFIRBW);
    }
    if (deviceSettingsKeys.contains("lpfFIRlog2Decim") || force) {
        swgZcuadrv9009InputSettings->setLpfFiRlog2Decim(settings.m_lpfFIRlog2Decim);
    }
    if (deviceSettingsKeys.contains("lpfFIRGain") || force) {
        swgZcuadrv9009InputSettings->setLpfFirGain(settings.m_lpfFIRGain);
    }
    if (deviceSettingsKeys.contains("fcPos") || force) {
        swgZcuadrv9009InputSettings->setFcPos((int) settings.m_fcPos);
    }
    if (deviceSettingsKeys.contains("dcBlock") || force) {
        swgZcuadrv9009InputSettings->setDcBlock(settings.m_dcBlock ? 1 : 0);
    }
    if (deviceSettingsKeys.contains("iqCorrection") || force) {
        swgZcuadrv9009InputSettings->setIqCorrection(settings.m_iqCorrection ? 1 : 0);
    }
    if (deviceSettingsKeys.contains("hwBBDCBlock") || force) {
        swgZcuadrv9009InputSettings->setHwBbdcBlock(settings.m_hwBBDCBlock ? 1 : 0);
    }
    if (deviceSettingsKeys.contains("hwRFDCBlock") || force) {
        swgZcuadrv9009InputSettings->setHwRfdcBlock(settings.m_hwRFDCBlock ? 1 : 0);
    }
    if (deviceSettingsKeys.contains("hwIQCorrection") || force) {
        swgZcuadrv9009InputSettings->setHwIqCorrection(settings.m_hwIQCorrection ? 1 : 0);
    }
    if (deviceSettingsKeys.contains("log2Decim") || force) {
        swgZcuadrv9009InputSettings->setLog2Decim(settings.m_log2Decim);
    }
    if (deviceSettingsKeys.contains("iqOrder") || force) {
        swgZcuadrv9009InputSettings->setIqOrder(settings.m_iqOrder ? 1 : 0);
    }
    if (deviceSettingsKeys.contains("lpfBW") || force) {
        swgZcuadrv9009InputSettings->setLpfBw(settings.m_lpfBW);
    }
    if (deviceSettingsKeys.contains("gain") || force) {
        swgZcuadrv9009InputSettings->setGain(settings.m_gain);
    }
    if (deviceSettingsKeys.contains("antennaPath") || force) {
        swgZcuadrv9009InputSettings->setAntennaPath((int) settings.m_antennaPath);
    }
    if (deviceSettingsKeys.contains("gainMode") || force) {
        swgZcuadrv9009InputSettings->setGainMode((int) settings.m_gainMode);
    }
    if (deviceSettingsKeys.contains("transverterDeltaFrequency") || force) {
        swgZcuadrv9009InputSettings->setTransverterDeltaFrequency(settings.m_transverterDeltaFrequency);
    }
    if (deviceSettingsKeys.contains("transverterMode") || force) {
        swgZcuadrv9009InputSettings->setTransverterMode(settings.m_transverterMode ? 1 : 0);
    }

    QString deviceSettingsURL = QString("http://%1:%2/sdrangel/deviceset/%3/device/settings")
            .arg(settings.m_reverseAPIAddress)
            .arg(settings.m_reverseAPIPort)
            .arg(settings.m_reverseAPIDeviceIndex);
    m_networkRequest.setUrl(QUrl(deviceSettingsURL));
    m_networkRequest.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    QBuffer *buffer = new QBuffer();
    buffer->open((QBuffer::ReadWrite));
    buffer->write(swgDeviceSettings->asJson().toUtf8());
    buffer->seek(0);

    // Always use PATCH to avoid passing reverse API settings
    QNetworkReply *reply = m_networkManager->sendCustomRequest(m_networkRequest, "PATCH", buffer);
    buffer->setParent(reply);

    delete swgDeviceSettings;
}

void Zcuadrv9009Input::webapiReverseSendStartStop(bool start)
{
    SWGSDRangel::SWGDeviceSettings *swgDeviceSettings = new SWGSDRangel::SWGDeviceSettings();
    swgDeviceSettings->setDirection(0); // single Rx
    swgDeviceSettings->setOriginatorIndex(m_deviceAPI->getDeviceSetIndex());
    swgDeviceSettings->setDeviceHwType(new QString("Zcuadrv9009"));

    QString deviceSettingsURL = QString("http://%1:%2/sdrangel/deviceset/%3/device/run")
            .arg(m_settings.m_reverseAPIAddress)
            .arg(m_settings.m_reverseAPIPort)
            .arg(m_settings.m_reverseAPIDeviceIndex);
    m_networkRequest.setUrl(QUrl(deviceSettingsURL));
    m_networkRequest.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    QBuffer *buffer = new QBuffer();
    buffer->open((QBuffer::ReadWrite));
    buffer->write(swgDeviceSettings->asJson().toUtf8());
    buffer->seek(0);
    QNetworkReply *reply;

    if (start) {
        reply = m_networkManager->sendCustomRequest(m_networkRequest, "POST", buffer);
    } else {
        reply = m_networkManager->sendCustomRequest(m_networkRequest, "DELETE", buffer);
    }

    buffer->setParent(reply);
    delete swgDeviceSettings;
}

void Zcuadrv9009Input::networkManagerFinished(QNetworkReply *reply)
{
    QNetworkReply::NetworkError replyError = reply->error();

    if (replyError)
    {
        qWarning() << "Zcuadrv9009Input::networkManagerFinished:"
                << " error(" << (int) replyError
                << "): " << replyError
                << ": " << reply->errorString();
    }
    else
    {
        QString answer = reply->readAll();
        answer.chop(1); // remove last \n
        qDebug("Zcuadrv9009Input::networkManagerFinished: reply:\n%s", answer.toStdString().c_str());
    }

    reply->deleteLater();
}
