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

#include <stdio.h>
#include <QDebug>
#include <QMessageBox>
#include <QFileDialog>

#include "dsp/dspengine.h"
#include "dsp/dspcommands.h"
#include "gui/glspectrum.h"
#include "gui/basicdevicesettingsdialog.h"
#include "gui/dialpopup.h"
#include "gui/dialogpositioner.h"
#include "device/deviceapi.h"
#include "device/deviceuiset.h"
#include "zcuadrv9009/devicezcuadrv9009.h"
#include "zcuadrv9009input.h"
#include "ui_zcuadrv9009inputgui.h"
#include "zcuadrv9009inputgui.h"

Zcuadrv9009InputGui::Zcuadrv9009InputGui(DeviceUISet *deviceUISet, QWidget* parent) :
    DeviceGUI(parent),
    ui(new Ui::Zcuadrv9009InputGUI),
    m_settings(),
    m_sampleRateMode(true),
    m_forceSettings(true),
    m_sampleSource(NULL),
    m_sampleRate(0),
    m_deviceCenterFrequency(0),
    m_lastEngineState(DeviceAPI::StNotStarted),
    m_doApplySettings(true),
    m_statusCounter(0)
{
    m_deviceUISet = deviceUISet;
    setAttribute(Qt::WA_DeleteOnClose, true);
    m_sampleSource = (Zcuadrv9009Input*) m_deviceUISet->m_deviceAPI->getSampleSource();

    ui->setupUi(getContents());
    sizeToContents();
    getContents()->setStyleSheet("#Zcuadrv9009InputGUI { background-color: rgb(64, 64, 64); }");
    m_helpURL = "plugins/samplesource/zcuadrv9009input/readme.md";
    ui->centerFrequency->setColorMapper(ColorMapper(ColorMapper::GrayGold));
    updateFrequencyLimits();

    ui->sampleRate->setColorMapper(ColorMapper(ColorMapper::GrayGreenYellow));
    ui->sampleRate->setValueRange(8, DeviceZcuadrv9009::srLowLimitFreq, DeviceZcuadrv9009::srHighLimitFreq);

    ui->lpf->setColorMapper(ColorMapper(ColorMapper::GrayYellow));

    quint32 minLimit, maxLimit;
    ((Zcuadrv9009Input *) m_sampleSource)->getbbLPRange(minLimit, maxLimit);
    ui->lpf->setValueRange(5, minLimit/1000, maxLimit/1000);

    ui->lpFIR->setColorMapper(ColorMapper(ColorMapper::GrayYellow));
    ui->lpFIR->setValueRange(5, 1U, 56000U); // will be dynamically recalculated

    ui->swDecimLabel->setText(QString::fromUtf8("S\u2193"));
    ui->lpFIRDecimationLabel->setText(QString::fromUtf8("\u2193"));

    connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(openDeviceSettingsDialog(const QPoint &)));

    blockApplySettings(true);
    displaySettings();
    makeUIConnections();
    blockApplySettings(false);

    connect(&m_updateTimer, SIGNAL(timeout()), this, SLOT(updateHardware()));
    connect(&m_statusTimer, SIGNAL(timeout()), this, SLOT(updateStatus()));
    m_statusTimer.start(500);

    connect(&m_inputMessageQueue, SIGNAL(messageEnqueued()), this, SLOT(handleInputMessages()), Qt::QueuedConnection);
    m_sampleSource->setMessageQueueToGUI(&m_inputMessageQueue);
    DialPopup::addPopupsToChildDials(this);
}

Zcuadrv9009InputGui::~Zcuadrv9009InputGui()
{
    m_statusTimer.stop();
    m_updateTimer.stop();
    delete ui;
}

void Zcuadrv9009InputGui::destroy()
{
    delete this;
}

void Zcuadrv9009InputGui::resetToDefaults()
{
	m_settings.resetToDefaults();
	displaySettings();
    m_forceSettings = true;
	sendSettings();
}

QByteArray Zcuadrv9009InputGui::serialize() const
{
    return m_settings.serialize();
}

bool Zcuadrv9009InputGui::deserialize(const QByteArray& data)
{
    if(m_settings.deserialize(data))
    {
        blockApplySettings(true);
        displaySettings();
        blockApplySettings(false);
        m_forceSettings = true;
        sendSettings(true);
        return true;
    }
    else
    {
        resetToDefaults();
        return false;
    }
}

bool Zcuadrv9009InputGui::handleMessage(const Message& message)
{
    if (Zcuadrv9009Input::MsgConfigureZcuadrv9009::match(message))
    {
        const Zcuadrv9009Input::MsgConfigureZcuadrv9009& cfg = (Zcuadrv9009Input::MsgConfigureZcuadrv9009&) message;

        if (cfg.getForce()) {
            m_settings = cfg.getSettings();
        } else {
            m_settings.applySettings(cfg.getSettingsKeys(), cfg.getSettings());
        }

        blockApplySettings(true);
        displaySettings();
        blockApplySettings(false);
        return true;
    }
    else if (Zcuadrv9009Input::MsgStartStop::match(message))
    {
        Zcuadrv9009Input::MsgStartStop& notif = (Zcuadrv9009Input::MsgStartStop&) message;
        blockApplySettings(true);
        ui->startStop->setChecked(notif.getStartStop());
        blockApplySettings(false);

        return true;
    }
    else if (DeviceZcuadrv9009Shared::MsgCrossReportToBuddy::match(message)) // message from buddy
    {
        DeviceZcuadrv9009Shared::MsgCrossReportToBuddy& conf = (DeviceZcuadrv9009Shared::MsgCrossReportToBuddy&) message;
        m_settings.m_devSampleRate = conf.getDevSampleRate();
        m_settings.m_lpfFIRlog2Decim = conf.getLpfFiRlog2IntDec();
        m_settings.m_lpfFIRBW = conf.getLpfFirbw();
        m_settings.m_LOppmTenths = conf.getLoPPMTenths();
        m_settings.m_lpfFIREnable = conf.isLpfFirEnable();
        blockApplySettings(true);
        displaySettings();
        blockApplySettings(false);

        return true;
    }
    else
    {
        return false;
    }
}

void Zcuadrv9009InputGui::on_startStop_toggled(bool checked)
{
    if (m_doApplySettings)
    {
        Zcuadrv9009Input::MsgStartStop *message = Zcuadrv9009Input::MsgStartStop::create(checked);
        m_sampleSource->getInputMessageQueue()->push(message);
    }
}

void Zcuadrv9009InputGui::on_centerFrequency_changed(quint64 value)
{
    m_settings.m_centerFrequency = value * 1000;
    m_settingsKeys.append("centerFrequency");
    sendSettings();
}

void Zcuadrv9009InputGui::on_loPPM_valueChanged(int value)
{
    ui->loPPMText->setText(QString("%1").arg(QString::number(value/10.0, 'f', 1)));
    m_settings.m_LOppmTenths = value;
    m_settingsKeys.append("LOppmTenths");
    sendSettings();
}

void Zcuadrv9009InputGui::on_dcOffset_toggled(bool checked)
{
    m_settings.m_dcBlock = checked;
    m_settingsKeys.append("dcBlock");
    sendSettings();
}

void Zcuadrv9009InputGui::on_iqImbalance_toggled(bool checked)
{
    m_settings.m_iqCorrection = checked;
    m_settingsKeys.append("iqCorrection");
    sendSettings();
}

void Zcuadrv9009InputGui::on_rfDCOffset_toggled(bool checked)
{
    m_settings.m_hwRFDCBlock = checked;
    m_settingsKeys.append("hwRFDCBlock");
    sendSettings();
}

void Zcuadrv9009InputGui::on_bbDCOffset_toggled(bool checked)
{
    m_settings.m_hwBBDCBlock = checked;
    m_settingsKeys.append("hwBBDCBlock");
    sendSettings();
}

void Zcuadrv9009InputGui::on_hwIQImbalance_toggled(bool checked)
{
    m_settings.m_hwIQCorrection = checked;
    m_settingsKeys.append("hwIQCorrection");
    sendSettings();
}


void Zcuadrv9009InputGui::on_swDecim_currentIndexChanged(int index)
{
    m_settings.m_log2Decim = index > 6 ? 6 : index;
    displaySampleRate();
    m_settings.m_devSampleRate = ui->sampleRate->getValueNew();

    if (!m_sampleRateMode) {
        m_settings.m_devSampleRate <<= m_settings.m_log2Decim;
    }

    m_settingsKeys.append("log2Decim");
    m_settingsKeys.append("devSampleRate");
    sendSettings();
}

void Zcuadrv9009InputGui::on_fcPos_currentIndexChanged(int index)
{
    m_settings.m_fcPos = (Zcuadrv9009InputSettings::fcPos_t) (index < (int) Zcuadrv9009InputSettings::FC_POS_END ? index : Zcuadrv9009InputSettings::FC_POS_CENTER);
    displayFcTooltip();
    m_settingsKeys.append("fcPos");
    sendSettings();
}

void Zcuadrv9009InputGui::on_sampleRate_changed(quint64 value)
{
    m_settings.m_devSampleRate = value;

    if (!m_sampleRateMode) {
        m_settings.m_devSampleRate <<= m_settings.m_log2Decim;
    }

    displayFcTooltip();
    m_settingsKeys.append("devSampleRate");
    sendSettings();
}

void Zcuadrv9009InputGui::on_lpf_changed(quint64 value)
{
    m_settings.m_lpfBW = value * 1000;
    m_settingsKeys.append("lpfBW");
    sendSettings();
}

void Zcuadrv9009InputGui::on_lpFIREnable_toggled(bool checked)
{
    m_settings.m_lpfFIREnable = checked;
    ui->lpFIRDecimation->setEnabled(checked);
    ui->lpFIRGain->setEnabled(checked);
    m_settingsKeys.append("lpfFIREnable");
    sendSettings();
}

void Zcuadrv9009InputGui::on_lpFIR_changed(quint64 value)
{
    m_settings.m_lpfFIRBW = value * 1000;
    m_settingsKeys.append("lpfFIRBW");
    sendSettings();
}

void Zcuadrv9009InputGui::on_lpFIRDecimation_currentIndexChanged(int index)
{
    m_settings.m_lpfFIRlog2Decim = index > 2 ? 2 : index;
    setSampleRateLimits();
    m_settingsKeys.append("lpfFIRlog2Decim");
    sendSettings();
}

void Zcuadrv9009InputGui::on_lpFIRGain_currentIndexChanged(int index)
{
    m_settings.m_lpfFIRGain = 6*(index > 3 ? 3 : index) - 12;
    m_settingsKeys.append("lpfFIRGain");
    sendSettings();
}

void Zcuadrv9009InputGui::on_gainMode_currentIndexChanged(int index)
{
    m_settings.m_gainMode = (Zcuadrv9009InputSettings::GainMode) (index < Zcuadrv9009InputSettings::GAIN_END ? index : 0);
    ui->gain->setEnabled(m_settings.m_gainMode == Zcuadrv9009InputSettings::GAIN_MANUAL);
    m_settingsKeys.append("gainMode");
    sendSettings();
}

void Zcuadrv9009InputGui::on_gain_valueChanged(int value)
{
    ui->gainText->setText(tr("%1").arg(value));
    m_settings.m_gain = value;
    m_settingsKeys.append("gain");
    sendSettings();
}

void Zcuadrv9009InputGui::on_antenna_currentIndexChanged(int index)
{
    m_settings.m_antennaPath = (Zcuadrv9009InputSettings::RFPath) (index < Zcuadrv9009InputSettings::RFPATH_END ? index : 0);
    m_settingsKeys.append("antennaPath");
    sendSettings();
}

void Zcuadrv9009InputGui::on_transverter_clicked()
{
    m_settings.m_transverterMode = ui->transverter->getDeltaFrequencyAcive();
    m_settings.m_transverterDeltaFrequency = ui->transverter->getDeltaFrequency();
    m_settings.m_iqOrder = ui->transverter->getIQOrder();
    qDebug("Zcuadrv9009InputGui::on_transverter_clicked: %lld Hz %s", m_settings.m_transverterDeltaFrequency, m_settings.m_transverterMode ? "on" : "off");
    updateFrequencyLimits();
    m_settings.m_centerFrequency = ui->centerFrequency->getValueNew()*1000;
    m_settingsKeys.append("transverterMode");
    m_settingsKeys.append("transverterDeltaFrequency");
    m_settingsKeys.append("iqOrder");
    m_settingsKeys.append("centerFrequency");
    sendSettings();
}

void Zcuadrv9009InputGui::on_sampleRateMode_toggled(bool checked)
{
    m_sampleRateMode = checked;
    displaySampleRate();
}

void Zcuadrv9009InputGui::displaySampleRate()
{
    ui->sampleRate->blockSignals(true);
    displayFcTooltip();

    if (m_sampleRateMode)
    {
        ui->sampleRateMode->setStyleSheet("QToolButton { background:rgb(60,60,60); }");
        ui->sampleRateMode->setText("SR");
        ui->sampleRate->setValueRange(8, DeviceZcuadrv9009::srLowLimitFreq, DeviceZcuadrv9009::srHighLimitFreq);
        ui->sampleRate->setValue(m_settings.m_devSampleRate);
        ui->sampleRate->setToolTip("Device to host sample rate (S/s)");
        ui->deviceRateText->setToolTip("Baseband sample rate (S/s)");
        uint32_t basebandSampleRate = m_settings.m_devSampleRate/(1<<m_settings.m_log2Decim);
        ui->deviceRateText->setText(tr("%1k").arg(QString::number(basebandSampleRate / 1000.0f, 'g', 5)));
    }
    else
    {
        ui->sampleRateMode->setStyleSheet("QToolButton { background:rgb(50,50,50); }");
        ui->sampleRateMode->setText("BB");
        ui->sampleRate->setValueRange(8, DeviceZcuadrv9009::srLowLimitFreq/(1<<m_settings.m_log2Decim), DeviceZcuadrv9009::srHighLimitFreq/(1<<m_settings.m_log2Decim));
        ui->sampleRate->setValue(m_settings.m_devSampleRate/(1<<m_settings.m_log2Decim));
        ui->sampleRate->setToolTip("Baseband sample rate (S/s)");
        ui->deviceRateText->setToolTip("Device to host sample rate (S/s)");
        ui->deviceRateText->setText(tr("%1k").arg(QString::number(m_settings.m_devSampleRate / 1000.0f, 'g', 5)));
    }

    ui->sampleRate->blockSignals(false);
}

void Zcuadrv9009InputGui::displayFcTooltip()
{
    int32_t fShift = DeviceSampleSource::calculateFrequencyShift(
        m_settings.m_log2Decim,
        (DeviceSampleSource::fcPos_t) m_settings.m_fcPos,
        m_settings.m_devSampleRate,
        DeviceSampleSource::FrequencyShiftScheme::FSHIFT_STD
    );
    ui->fcPos->setToolTip(tr("Relative position of device center frequency: %1 kHz").arg(QString::number(fShift / 1000.0f, 'g', 5)));
}

void Zcuadrv9009InputGui::displaySettings()
{
    ui->transverter->setDeltaFrequency(m_settings.m_transverterDeltaFrequency);
    ui->transverter->setDeltaFrequencyActive(m_settings.m_transverterMode);
    ui->transverter->setIQOrder(m_settings.m_iqOrder);
    updateFrequencyLimits();
    ui->centerFrequency->setValue(m_settings.m_centerFrequency / 1000);
    displaySampleRate();

    ui->dcOffset->setChecked(m_settings.m_dcBlock);
    ui->iqImbalance->setChecked(m_settings.m_iqCorrection);
    ui->bbDCOffset->setChecked(m_settings.m_hwBBDCBlock);
    ui->rfDCOffset->setChecked(m_settings.m_hwRFDCBlock);
    ui->hwIQImbalance->setChecked(m_settings.m_hwIQCorrection);
    ui->loPPM->setValue(m_settings.m_LOppmTenths);
    ui->loPPMText->setText(QString("%1").arg(QString::number(m_settings.m_LOppmTenths/10.0, 'f', 1)));

    ui->swDecim->setCurrentIndex(m_settings.m_log2Decim);
    ui->fcPos->setCurrentIndex((int) m_settings.m_fcPos);

    ui->lpf->setValue(m_settings.m_lpfBW / 1000);

    ui->lpFIREnable->setChecked(m_settings.m_lpfFIREnable);
    ui->lpFIR->setValue(m_settings.m_lpfFIRBW / 1000);
    ui->lpFIRDecimation->setCurrentIndex(m_settings.m_lpfFIRlog2Decim);
    ui->lpFIRGain->setCurrentIndex((m_settings.m_lpfFIRGain + 12)/6);
    ui->lpFIRDecimation->setEnabled(m_settings.m_lpfFIREnable);
    ui->lpFIRGain->setEnabled(m_settings.m_lpfFIREnable);

    ui->gainMode->setCurrentIndex((int) m_settings.m_gainMode);
    ui->gain->setValue(m_settings.m_gain);
    ui->gainText->setText(tr("%1").arg(m_settings.m_gain));

    ui->antenna->setCurrentIndex((int) m_settings.m_antennaPath);

    setFIRBWLimits();
    setSampleRateLimits();
}

void Zcuadrv9009InputGui::sendSettings(bool forceSettings)
{
    m_forceSettings = forceSettings;
    if(!m_updateTimer.isActive()) { m_updateTimer.start(100); }
}

void Zcuadrv9009InputGui::updateHardware()
{
    if (m_doApplySettings)
    {
        qDebug() << "Zcuadrv9009InputGui::updateHardware";
        Zcuadrv9009Input::MsgConfigureZcuadrv9009* message = Zcuadrv9009Input::MsgConfigureZcuadrv9009::create(m_settings, m_settingsKeys, m_forceSettings);
        m_sampleSource->getInputMessageQueue()->push(message);
        m_forceSettings = false;
        m_settingsKeys.clear();
        m_updateTimer.stop();
    }
}

void Zcuadrv9009InputGui::blockApplySettings(bool block)
{
    m_doApplySettings = !block;
}

void Zcuadrv9009InputGui::updateStatus()
{
    int state = m_deviceUISet->m_deviceAPI->state();

    if(m_lastEngineState != state)
    {
        switch(state)
        {
            case DeviceAPI::StNotStarted:
                ui->startStop->setStyleSheet("QToolButton { background:rgb(79,79,79); }");
                break;
            case DeviceAPI::StIdle:
                ui->startStop->setStyleSheet("QToolButton { background-color : blue; }");
                break;
            case DeviceAPI::StRunning:
                ui->startStop->setStyleSheet("QToolButton { background-color : green; }");
                break;
            case DeviceAPI::StError:
                ui->startStop->setStyleSheet("QToolButton { background-color : red; }");
                QMessageBox::information(this, tr("Message"), m_deviceUISet->m_deviceAPI->errorMessage());
                break;
            default:
                break;
        }

        m_lastEngineState = state;
    }

    if (m_statusCounter % 2 == 0) // 1s
    {
        uint32_t adcRate = ((Zcuadrv9009Input *) m_sampleSource)->getADCSampleRate();

        if (adcRate < 100000000) {
            ui->adcRateText->setText(tr("%1k").arg(QString::number(adcRate / 1000.0f, 'g', 5)));
        } else {
            ui->adcRateText->setText(tr("%1M").arg(QString::number(adcRate / 1000000.0f, 'g', 5)));
        }
    }

    if (m_statusCounter % 4 == 0) // 2s
    {
        std::string rssiStr;
        ((Zcuadrv9009Input *) m_sampleSource)->getRSSI(rssiStr);
        ui->rssiText->setText(tr("-%1").arg(QString::fromStdString(rssiStr)));
        int gaindB;
        ((Zcuadrv9009Input *) m_sampleSource)->getGain(gaindB);
        ui->actualGainText->setText(tr("%1").arg(gaindB));
    }

    if (m_statusCounter % 10 == 0) // 5s
    {
        if (m_deviceUISet->m_deviceAPI->isBuddyLeader()) {
            ((Zcuadrv9009Input *) m_sampleSource)->fetchTemperature();
        }

        ui->temperatureText->setText(tr("%1C").arg(QString::number(((Zcuadrv9009Input *) m_sampleSource)->getTemperature(), 'f', 0)));
    }

    m_statusCounter++;
}

void Zcuadrv9009InputGui::setFIRBWLimits()
{
    float high = DeviceZcuadrv9009::firBWHighLimitFactor * ((Zcuadrv9009Input *) m_sampleSource)->getFIRSampleRate();
    float low = DeviceZcuadrv9009::firBWLowLimitFactor * ((Zcuadrv9009Input *) m_sampleSource)->getFIRSampleRate();
    ui->lpFIR->setValueRange(5, (int(low)/1000)+1, (int(high)/1000)+1);
    ui->lpFIR->setValue(m_settings.m_lpfFIRBW/1000);
}

void Zcuadrv9009InputGui::setSampleRateLimits()
{
    uint32_t low = ui->lpFIREnable->isChecked() ? DeviceZcuadrv9009::srLowLimitFreq / (1<<ui->lpFIRDecimation->currentIndex()) : DeviceZcuadrv9009::srLowLimitFreq;
    ui->sampleRate->setValueRange(8, low, DeviceZcuadrv9009::srHighLimitFreq);
    ui->sampleRate->setValue(m_settings.m_devSampleRate);
}

void Zcuadrv9009InputGui::updateFrequencyLimits()
{
    qint64 minLimit, maxLimit;
    // values should be in kHz
    qint64 deltaFrequency = m_settings.m_transverterMode ? m_settings.m_transverterDeltaFrequency/1000 : 0;
    ((Zcuadrv9009Input *) m_sampleSource)->getLORange(minLimit, maxLimit);

    minLimit = minLimit/1000 + deltaFrequency;
    maxLimit = maxLimit/1000 + deltaFrequency;

    if (m_settings.m_transverterMode)
    {
        minLimit = minLimit < 0 ? 0 : minLimit > 999999999 ? 999999999 : minLimit;
        maxLimit = maxLimit < 0 ? 0 : maxLimit > 999999999 ? 999999999 : maxLimit;
        ui->centerFrequency->setValueRange(9, minLimit, maxLimit);
    }
    else
    {
        minLimit = minLimit < 0 ? 0 : minLimit > 9999999 ? 9999999 : minLimit;
        maxLimit = maxLimit < 0 ? 0 : maxLimit > 9999999 ? 9999999 : maxLimit;
        ui->centerFrequency->setValueRange(7, minLimit, maxLimit);
    }
    qDebug("Zcuadrv9009InputGui::updateFrequencyLimits: delta: %lld min: %lld max: %lld", deltaFrequency, minLimit, maxLimit);
}

void Zcuadrv9009InputGui::handleInputMessages()
{
    Message* message;

    while ((message = m_inputMessageQueue.pop()) != 0)
    {
        qDebug("Zcuadrv9009InputGui::handleInputMessages: message: %s", message->getIdentifier());

        if (DSPSignalNotification::match(*message))
        {
            DSPSignalNotification* notif = (DSPSignalNotification*) message;
            m_sampleRate = notif->getSampleRate();
            m_deviceCenterFrequency = notif->getCenterFrequency();
            qDebug("Zcuadrv9009InputGui::handleInputMessages: DSPSignalNotification: SampleRate: %d, CenterFrequency: %llu", notif->getSampleRate(), notif->getCenterFrequency());
            updateSampleRateAndFrequency();
            setFIRBWLimits();

            delete message;
        }
        else
        {
            if (handleMessage(*message))
            {
                delete message;
            }
        }
    }
}

void Zcuadrv9009InputGui::updateSampleRateAndFrequency()
{
    m_deviceUISet->getSpectrum()->setSampleRate(m_sampleRate);
    m_deviceUISet->getSpectrum()->setCenterFrequency(m_deviceCenterFrequency);
    displaySampleRate();
}

void Zcuadrv9009InputGui::openDeviceSettingsDialog(const QPoint& p)
{
    if (m_contextMenuType == ContextMenuDeviceSettings)
    {
        BasicDeviceSettingsDialog dialog(this);
        dialog.setUseReverseAPI(m_settings.m_useReverseAPI);
        dialog.setReverseAPIAddress(m_settings.m_reverseAPIAddress);
        dialog.setReverseAPIPort(m_settings.m_reverseAPIPort);
        dialog.setReverseAPIDeviceIndex(m_settings.m_reverseAPIDeviceIndex);

        dialog.move(p);
        new DialogPositioner(&dialog, false);
        dialog.exec();

        m_settings.m_useReverseAPI = dialog.useReverseAPI();
        m_settings.m_reverseAPIAddress = dialog.getReverseAPIAddress();
        m_settings.m_reverseAPIPort = dialog.getReverseAPIPort();
        m_settings.m_reverseAPIDeviceIndex = dialog.getReverseAPIDeviceIndex();
        m_settingsKeys.append("useReverseAPI");
        m_settingsKeys.append("reverseAPIAddress");
        m_settingsKeys.append("reverseAPIPort");
        m_settingsKeys.append("reverseAPIDeviceIndex");

        sendSettings();
    }

    resetContextMenuType();
}

void Zcuadrv9009InputGui::makeUIConnections()
{
    QObject::connect(ui->startStop, &ButtonSwitch::toggled, this, &Zcuadrv9009InputGui::on_startStop_toggled);
    QObject::connect(ui->centerFrequency, &ValueDial::changed, this, &Zcuadrv9009InputGui::on_centerFrequency_changed);
    QObject::connect(ui->loPPM, &QSlider::valueChanged, this, &Zcuadrv9009InputGui::on_loPPM_valueChanged);
    QObject::connect(ui->dcOffset, &ButtonSwitch::toggled, this, &Zcuadrv9009InputGui::on_dcOffset_toggled);
    QObject::connect(ui->rfDCOffset, &ButtonSwitch::toggled, this, &Zcuadrv9009InputGui::on_rfDCOffset_toggled);
    QObject::connect(ui->bbDCOffset, &ButtonSwitch::toggled, this, &Zcuadrv9009InputGui::on_bbDCOffset_toggled);
    QObject::connect(ui->hwIQImbalance, &ButtonSwitch::toggled, this, &Zcuadrv9009InputGui::on_hwIQImbalance_toggled);
    QObject::connect(ui->iqImbalance, &ButtonSwitch::toggled, this, &Zcuadrv9009InputGui::on_iqImbalance_toggled);
    QObject::connect(ui->swDecim, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &Zcuadrv9009InputGui::on_swDecim_currentIndexChanged);
    QObject::connect(ui->fcPos, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &Zcuadrv9009InputGui::on_fcPos_currentIndexChanged);
    QObject::connect(ui->sampleRate, &ValueDial::changed, this, &Zcuadrv9009InputGui::on_sampleRate_changed);
    QObject::connect(ui->lpf, &ValueDial::changed, this, &Zcuadrv9009InputGui::on_lpf_changed);
    QObject::connect(ui->lpFIREnable, &ButtonSwitch::toggled, this, &Zcuadrv9009InputGui::on_lpFIREnable_toggled);
    QObject::connect(ui->lpFIR, &ValueDial::changed, this, &Zcuadrv9009InputGui::on_lpFIR_changed);
    QObject::connect(ui->lpFIRDecimation, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &Zcuadrv9009InputGui::on_lpFIRDecimation_currentIndexChanged);
    QObject::connect(ui->lpFIRGain, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &Zcuadrv9009InputGui::on_lpFIRGain_currentIndexChanged);
    QObject::connect(ui->gainMode, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &Zcuadrv9009InputGui::on_gainMode_currentIndexChanged);
    QObject::connect(ui->gain, &QDial::valueChanged, this, &Zcuadrv9009InputGui::on_gain_valueChanged);
    QObject::connect(ui->antenna, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &Zcuadrv9009InputGui::on_antenna_currentIndexChanged);
    QObject::connect(ui->transverter, &TransverterButton::clicked, this, &Zcuadrv9009InputGui::on_transverter_clicked);
    QObject::connect(ui->sampleRateMode, &QToolButton::toggled, this, &Zcuadrv9009InputGui::on_sampleRateMode_toggled);
}
