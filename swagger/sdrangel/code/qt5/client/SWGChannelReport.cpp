/**
 * SDRangel
 * This is the web REST/JSON API of SDRangel SDR software. SDRangel is an Open Source Qt5/OpenGL 3.0+ (4.3+ in Windows) GUI and server Software Defined Radio and signal analyzer in software. It supports Airspy, BladeRF, HackRF, LimeSDR, PlutoSDR, RTL-SDR, SDRplay RSP1 and FunCube    ---   Limitations and specifcities:    * In SDRangel GUI the first Rx device set cannot be deleted. Conversely the server starts with no device sets and its number of device sets can be reduced to zero by as many calls as necessary to /sdrangel/deviceset with DELETE method.   * Preset import and export from/to file is a server only feature.   * Device set focus is a GUI only feature.   * The following channels are not implemented (status 501 is returned): ATV and DATV demodulators, Channel Analyzer NG, LoRa demodulator   * The device settings and report structures contains only the sub-structure corresponding to the device type. The DeviceSettings and DeviceReport structures documented here shows all of them but only one will be or should be present at a time   * The channel settings and report structures contains only the sub-structure corresponding to the channel type. The ChannelSettings and ChannelReport structures documented here shows all of them but only one will be or should be present at a time    --- 
 *
 * OpenAPI spec version: 6.0.0
 * Contact: f4exb06@gmail.com
 *
 * NOTE: This class is auto generated by the swagger code generator program.
 * https://github.com/swagger-api/swagger-codegen.git
 * Do not edit the class manually.
 */


#include "SWGChannelReport.h"

#include "SWGHelpers.h"

#include <QJsonDocument>
#include <QJsonArray>
#include <QObject>
#include <QDebug>

namespace SWGSDRangel {

SWGChannelReport::SWGChannelReport(QString* json) {
    init();
    this->fromJson(*json);
}

SWGChannelReport::SWGChannelReport() {
    channel_type = nullptr;
    m_channel_type_isSet = false;
    direction = 0;
    m_direction_isSet = false;
    adsb_demod_report = nullptr;
    m_adsb_demod_report_isSet = false;
    ais_demod_report = nullptr;
    m_ais_demod_report_isSet = false;
    ais_mod_report = nullptr;
    m_ais_mod_report_isSet = false;
    am_demod_report = nullptr;
    m_am_demod_report_isSet = false;
    am_mod_report = nullptr;
    m_am_mod_report_isSet = false;
    atv_mod_report = nullptr;
    m_atv_mod_report_isSet = false;
    bfm_demod_report = nullptr;
    m_bfm_demod_report_isSet = false;
    chirp_chat_demod_report = nullptr;
    m_chirp_chat_demod_report_isSet = false;
    chirp_chat_mod_report = nullptr;
    m_chirp_chat_mod_report_isSet = false;
    datv_demod_report = nullptr;
    m_datv_demod_report_isSet = false;
    datv_mod_report = nullptr;
    m_datv_mod_report_isSet = false;
    dsd_demod_report = nullptr;
    m_dsd_demod_report_isSet = false;
    ieee_802_15_4_mod_report = nullptr;
    m_ieee_802_15_4_mod_report_isSet = false;
    file_sink_report = nullptr;
    m_file_sink_report_isSet = false;
    file_source_report = nullptr;
    m_file_source_report_isSet = false;
    free_dv_demod_report = nullptr;
    m_free_dv_demod_report_isSet = false;
    free_dv_mod_report = nullptr;
    m_free_dv_mod_report_isSet = false;
    freq_tracker_report = nullptr;
    m_freq_tracker_report_isSet = false;
    nfm_demod_report = nullptr;
    m_nfm_demod_report_isSet = false;
    nfm_mod_report = nullptr;
    m_nfm_mod_report_isSet = false;
    noise_figure_report = nullptr;
    m_noise_figure_report_isSet = false;
    ssb_demod_report = nullptr;
    m_ssb_demod_report_isSet = false;
    radio_astronomy_report = nullptr;
    m_radio_astronomy_report_isSet = false;
    radio_clock_report = nullptr;
    m_radio_clock_report_isSet = false;
    remote_source_report = nullptr;
    m_remote_source_report_isSet = false;
    packet_demod_report = nullptr;
    m_packet_demod_report_isSet = false;
    packet_mod_report = nullptr;
    m_packet_mod_report_isSet = false;
    pager_demod_report = nullptr;
    m_pager_demod_report_isSet = false;
    sig_mf_file_sink_report = nullptr;
    m_sig_mf_file_sink_report_isSet = false;
    ssb_mod_report = nullptr;
    m_ssb_mod_report_isSet = false;
    udp_source_report = nullptr;
    m_udp_source_report_isSet = false;
    udp_sink_report = nullptr;
    m_udp_sink_report_isSet = false;
    vor_demod_report = nullptr;
    m_vor_demod_report_isSet = false;
    vor_demod_sc_report = nullptr;
    m_vor_demod_sc_report_isSet = false;
    wfm_demod_report = nullptr;
    m_wfm_demod_report_isSet = false;
    wfm_mod_report = nullptr;
    m_wfm_mod_report_isSet = false;
}

SWGChannelReport::~SWGChannelReport() {
    this->cleanup();
}

void
SWGChannelReport::init() {
    channel_type = new QString("");
    m_channel_type_isSet = false;
    direction = 0;
    m_direction_isSet = false;
    adsb_demod_report = new SWGADSBDemodReport();
    m_adsb_demod_report_isSet = false;
    ais_demod_report = new SWGAISDemodReport();
    m_ais_demod_report_isSet = false;
    ais_mod_report = new SWGAISModReport();
    m_ais_mod_report_isSet = false;
    am_demod_report = new SWGAMDemodReport();
    m_am_demod_report_isSet = false;
    am_mod_report = new SWGAMModReport();
    m_am_mod_report_isSet = false;
    atv_mod_report = new SWGATVModReport();
    m_atv_mod_report_isSet = false;
    bfm_demod_report = new SWGBFMDemodReport();
    m_bfm_demod_report_isSet = false;
    chirp_chat_demod_report = new SWGChirpChatDemodReport();
    m_chirp_chat_demod_report_isSet = false;
    chirp_chat_mod_report = new SWGChirpChatModReport();
    m_chirp_chat_mod_report_isSet = false;
    datv_demod_report = new SWGDATVDemodReport();
    m_datv_demod_report_isSet = false;
    datv_mod_report = new SWGDATVModReport();
    m_datv_mod_report_isSet = false;
    dsd_demod_report = new SWGDSDDemodReport();
    m_dsd_demod_report_isSet = false;
    ieee_802_15_4_mod_report = new SWGIEEE_802_15_4_ModReport();
    m_ieee_802_15_4_mod_report_isSet = false;
    file_sink_report = new SWGFileSinkReport();
    m_file_sink_report_isSet = false;
    file_source_report = new SWGFileSourceReport();
    m_file_source_report_isSet = false;
    free_dv_demod_report = new SWGFreeDVDemodReport();
    m_free_dv_demod_report_isSet = false;
    free_dv_mod_report = new SWGFreeDVModReport();
    m_free_dv_mod_report_isSet = false;
    freq_tracker_report = new SWGFreqTrackerReport();
    m_freq_tracker_report_isSet = false;
    nfm_demod_report = new SWGNFMDemodReport();
    m_nfm_demod_report_isSet = false;
    nfm_mod_report = new SWGNFMModReport();
    m_nfm_mod_report_isSet = false;
    noise_figure_report = new SWGNoiseFigureReport();
    m_noise_figure_report_isSet = false;
    ssb_demod_report = new SWGSSBDemodReport();
    m_ssb_demod_report_isSet = false;
    radio_astronomy_report = new SWGRadioAstronomyReport();
    m_radio_astronomy_report_isSet = false;
    radio_clock_report = new SWGRadioClockReport();
    m_radio_clock_report_isSet = false;
    remote_source_report = new SWGRemoteSourceReport();
    m_remote_source_report_isSet = false;
    packet_demod_report = new SWGPacketDemodReport();
    m_packet_demod_report_isSet = false;
    packet_mod_report = new SWGPacketModReport();
    m_packet_mod_report_isSet = false;
    pager_demod_report = new SWGPagerDemodReport();
    m_pager_demod_report_isSet = false;
    sig_mf_file_sink_report = new SWGSigMFFileSinkReport();
    m_sig_mf_file_sink_report_isSet = false;
    ssb_mod_report = new SWGSSBModReport();
    m_ssb_mod_report_isSet = false;
    udp_source_report = new SWGUDPSourceReport();
    m_udp_source_report_isSet = false;
    udp_sink_report = new SWGUDPSinkReport();
    m_udp_sink_report_isSet = false;
    vor_demod_report = new SWGVORDemodReport();
    m_vor_demod_report_isSet = false;
    vor_demod_sc_report = new SWGVORDemodSCReport();
    m_vor_demod_sc_report_isSet = false;
    wfm_demod_report = new SWGWFMDemodReport();
    m_wfm_demod_report_isSet = false;
    wfm_mod_report = new SWGWFMModReport();
    m_wfm_mod_report_isSet = false;
}

void
SWGChannelReport::cleanup() {
    if(channel_type != nullptr) { 
        delete channel_type;
    }

    if(adsb_demod_report != nullptr) { 
        delete adsb_demod_report;
    }
    if(ais_demod_report != nullptr) { 
        delete ais_demod_report;
    }
    if(ais_mod_report != nullptr) { 
        delete ais_mod_report;
    }
    if(am_demod_report != nullptr) { 
        delete am_demod_report;
    }
    if(am_mod_report != nullptr) { 
        delete am_mod_report;
    }
    if(atv_mod_report != nullptr) { 
        delete atv_mod_report;
    }
    if(bfm_demod_report != nullptr) { 
        delete bfm_demod_report;
    }
    if(chirp_chat_demod_report != nullptr) { 
        delete chirp_chat_demod_report;
    }
    if(chirp_chat_mod_report != nullptr) { 
        delete chirp_chat_mod_report;
    }
    if(datv_demod_report != nullptr) { 
        delete datv_demod_report;
    }
    if(datv_mod_report != nullptr) { 
        delete datv_mod_report;
    }
    if(dsd_demod_report != nullptr) { 
        delete dsd_demod_report;
    }
    if(ieee_802_15_4_mod_report != nullptr) { 
        delete ieee_802_15_4_mod_report;
    }
    if(file_sink_report != nullptr) { 
        delete file_sink_report;
    }
    if(file_source_report != nullptr) { 
        delete file_source_report;
    }
    if(free_dv_demod_report != nullptr) { 
        delete free_dv_demod_report;
    }
    if(free_dv_mod_report != nullptr) { 
        delete free_dv_mod_report;
    }
    if(freq_tracker_report != nullptr) { 
        delete freq_tracker_report;
    }
    if(nfm_demod_report != nullptr) { 
        delete nfm_demod_report;
    }
    if(nfm_mod_report != nullptr) { 
        delete nfm_mod_report;
    }
    if(noise_figure_report != nullptr) { 
        delete noise_figure_report;
    }
    if(ssb_demod_report != nullptr) { 
        delete ssb_demod_report;
    }
    if(radio_astronomy_report != nullptr) { 
        delete radio_astronomy_report;
    }
    if(radio_clock_report != nullptr) { 
        delete radio_clock_report;
    }
    if(remote_source_report != nullptr) { 
        delete remote_source_report;
    }
    if(packet_demod_report != nullptr) { 
        delete packet_demod_report;
    }
    if(packet_mod_report != nullptr) { 
        delete packet_mod_report;
    }
    if(pager_demod_report != nullptr) { 
        delete pager_demod_report;
    }
    if(sig_mf_file_sink_report != nullptr) { 
        delete sig_mf_file_sink_report;
    }
    if(ssb_mod_report != nullptr) { 
        delete ssb_mod_report;
    }
    if(udp_source_report != nullptr) { 
        delete udp_source_report;
    }
    if(udp_sink_report != nullptr) { 
        delete udp_sink_report;
    }
    if(vor_demod_report != nullptr) { 
        delete vor_demod_report;
    }
    if(vor_demod_sc_report != nullptr) { 
        delete vor_demod_sc_report;
    }
    if(wfm_demod_report != nullptr) { 
        delete wfm_demod_report;
    }
    if(wfm_mod_report != nullptr) { 
        delete wfm_mod_report;
    }
}

SWGChannelReport*
SWGChannelReport::fromJson(QString &json) {
    QByteArray array (json.toStdString().c_str());
    QJsonDocument doc = QJsonDocument::fromJson(array);
    QJsonObject jsonObject = doc.object();
    this->fromJsonObject(jsonObject);
    return this;
}

void
SWGChannelReport::fromJsonObject(QJsonObject &pJson) {
    ::SWGSDRangel::setValue(&channel_type, pJson["channelType"], "QString", "QString");
    
    ::SWGSDRangel::setValue(&direction, pJson["direction"], "qint32", "");
    
    ::SWGSDRangel::setValue(&adsb_demod_report, pJson["ADSBDemodReport"], "SWGADSBDemodReport", "SWGADSBDemodReport");
    
    ::SWGSDRangel::setValue(&ais_demod_report, pJson["AISDemodReport"], "SWGAISDemodReport", "SWGAISDemodReport");
    
    ::SWGSDRangel::setValue(&ais_mod_report, pJson["AISModReport"], "SWGAISModReport", "SWGAISModReport");
    
    ::SWGSDRangel::setValue(&am_demod_report, pJson["AMDemodReport"], "SWGAMDemodReport", "SWGAMDemodReport");
    
    ::SWGSDRangel::setValue(&am_mod_report, pJson["AMModReport"], "SWGAMModReport", "SWGAMModReport");
    
    ::SWGSDRangel::setValue(&atv_mod_report, pJson["ATVModReport"], "SWGATVModReport", "SWGATVModReport");
    
    ::SWGSDRangel::setValue(&bfm_demod_report, pJson["BFMDemodReport"], "SWGBFMDemodReport", "SWGBFMDemodReport");
    
    ::SWGSDRangel::setValue(&chirp_chat_demod_report, pJson["ChirpChatDemodReport"], "SWGChirpChatDemodReport", "SWGChirpChatDemodReport");
    
    ::SWGSDRangel::setValue(&chirp_chat_mod_report, pJson["ChirpChatModReport"], "SWGChirpChatModReport", "SWGChirpChatModReport");
    
    ::SWGSDRangel::setValue(&datv_demod_report, pJson["DATVDemodReport"], "SWGDATVDemodReport", "SWGDATVDemodReport");
    
    ::SWGSDRangel::setValue(&datv_mod_report, pJson["DATVModReport"], "SWGDATVModReport", "SWGDATVModReport");
    
    ::SWGSDRangel::setValue(&dsd_demod_report, pJson["DSDDemodReport"], "SWGDSDDemodReport", "SWGDSDDemodReport");
    
    ::SWGSDRangel::setValue(&ieee_802_15_4_mod_report, pJson["IEEE_802_15_4_ModReport"], "SWGIEEE_802_15_4_ModReport", "SWGIEEE_802_15_4_ModReport");
    
    ::SWGSDRangel::setValue(&file_sink_report, pJson["FileSinkReport"], "SWGFileSinkReport", "SWGFileSinkReport");
    
    ::SWGSDRangel::setValue(&file_source_report, pJson["FileSourceReport"], "SWGFileSourceReport", "SWGFileSourceReport");
    
    ::SWGSDRangel::setValue(&free_dv_demod_report, pJson["FreeDVDemodReport"], "SWGFreeDVDemodReport", "SWGFreeDVDemodReport");
    
    ::SWGSDRangel::setValue(&free_dv_mod_report, pJson["FreeDVModReport"], "SWGFreeDVModReport", "SWGFreeDVModReport");
    
    ::SWGSDRangel::setValue(&freq_tracker_report, pJson["FreqTrackerReport"], "SWGFreqTrackerReport", "SWGFreqTrackerReport");
    
    ::SWGSDRangel::setValue(&nfm_demod_report, pJson["NFMDemodReport"], "SWGNFMDemodReport", "SWGNFMDemodReport");
    
    ::SWGSDRangel::setValue(&nfm_mod_report, pJson["NFMModReport"], "SWGNFMModReport", "SWGNFMModReport");
    
    ::SWGSDRangel::setValue(&noise_figure_report, pJson["NoiseFigureReport"], "SWGNoiseFigureReport", "SWGNoiseFigureReport");
    
    ::SWGSDRangel::setValue(&ssb_demod_report, pJson["SSBDemodReport"], "SWGSSBDemodReport", "SWGSSBDemodReport");
    
    ::SWGSDRangel::setValue(&radio_astronomy_report, pJson["RadioAstronomyReport"], "SWGRadioAstronomyReport", "SWGRadioAstronomyReport");
    
    ::SWGSDRangel::setValue(&radio_clock_report, pJson["RadioClockReport"], "SWGRadioClockReport", "SWGRadioClockReport");
    
    ::SWGSDRangel::setValue(&remote_source_report, pJson["RemoteSourceReport"], "SWGRemoteSourceReport", "SWGRemoteSourceReport");
    
    ::SWGSDRangel::setValue(&packet_demod_report, pJson["PacketDemodReport"], "SWGPacketDemodReport", "SWGPacketDemodReport");
    
    ::SWGSDRangel::setValue(&packet_mod_report, pJson["PacketModReport"], "SWGPacketModReport", "SWGPacketModReport");
    
    ::SWGSDRangel::setValue(&pager_demod_report, pJson["PagerDemodReport"], "SWGPagerDemodReport", "SWGPagerDemodReport");
    
    ::SWGSDRangel::setValue(&sig_mf_file_sink_report, pJson["SigMFFileSinkReport"], "SWGSigMFFileSinkReport", "SWGSigMFFileSinkReport");
    
    ::SWGSDRangel::setValue(&ssb_mod_report, pJson["SSBModReport"], "SWGSSBModReport", "SWGSSBModReport");
    
    ::SWGSDRangel::setValue(&udp_source_report, pJson["UDPSourceReport"], "SWGUDPSourceReport", "SWGUDPSourceReport");
    
    ::SWGSDRangel::setValue(&udp_sink_report, pJson["UDPSinkReport"], "SWGUDPSinkReport", "SWGUDPSinkReport");
    
    ::SWGSDRangel::setValue(&vor_demod_report, pJson["VORDemodReport"], "SWGVORDemodReport", "SWGVORDemodReport");
    
    ::SWGSDRangel::setValue(&vor_demod_sc_report, pJson["VORDemodSCReport"], "SWGVORDemodSCReport", "SWGVORDemodSCReport");
    
    ::SWGSDRangel::setValue(&wfm_demod_report, pJson["WFMDemodReport"], "SWGWFMDemodReport", "SWGWFMDemodReport");
    
    ::SWGSDRangel::setValue(&wfm_mod_report, pJson["WFMModReport"], "SWGWFMModReport", "SWGWFMModReport");
    
}

QString
SWGChannelReport::asJson ()
{
    QJsonObject* obj = this->asJsonObject();

    QJsonDocument doc(*obj);
    QByteArray bytes = doc.toJson();
    delete obj;
    return QString(bytes);
}

QJsonObject*
SWGChannelReport::asJsonObject() {
    QJsonObject* obj = new QJsonObject();
    if(channel_type != nullptr && *channel_type != QString("")){
        toJsonValue(QString("channelType"), channel_type, obj, QString("QString"));
    }
    if(m_direction_isSet){
        obj->insert("direction", QJsonValue(direction));
    }
    if((adsb_demod_report != nullptr) && (adsb_demod_report->isSet())){
        toJsonValue(QString("ADSBDemodReport"), adsb_demod_report, obj, QString("SWGADSBDemodReport"));
    }
    if((ais_demod_report != nullptr) && (ais_demod_report->isSet())){
        toJsonValue(QString("AISDemodReport"), ais_demod_report, obj, QString("SWGAISDemodReport"));
    }
    if((ais_mod_report != nullptr) && (ais_mod_report->isSet())){
        toJsonValue(QString("AISModReport"), ais_mod_report, obj, QString("SWGAISModReport"));
    }
    if((am_demod_report != nullptr) && (am_demod_report->isSet())){
        toJsonValue(QString("AMDemodReport"), am_demod_report, obj, QString("SWGAMDemodReport"));
    }
    if((am_mod_report != nullptr) && (am_mod_report->isSet())){
        toJsonValue(QString("AMModReport"), am_mod_report, obj, QString("SWGAMModReport"));
    }
    if((atv_mod_report != nullptr) && (atv_mod_report->isSet())){
        toJsonValue(QString("ATVModReport"), atv_mod_report, obj, QString("SWGATVModReport"));
    }
    if((bfm_demod_report != nullptr) && (bfm_demod_report->isSet())){
        toJsonValue(QString("BFMDemodReport"), bfm_demod_report, obj, QString("SWGBFMDemodReport"));
    }
    if((chirp_chat_demod_report != nullptr) && (chirp_chat_demod_report->isSet())){
        toJsonValue(QString("ChirpChatDemodReport"), chirp_chat_demod_report, obj, QString("SWGChirpChatDemodReport"));
    }
    if((chirp_chat_mod_report != nullptr) && (chirp_chat_mod_report->isSet())){
        toJsonValue(QString("ChirpChatModReport"), chirp_chat_mod_report, obj, QString("SWGChirpChatModReport"));
    }
    if((datv_demod_report != nullptr) && (datv_demod_report->isSet())){
        toJsonValue(QString("DATVDemodReport"), datv_demod_report, obj, QString("SWGDATVDemodReport"));
    }
    if((datv_mod_report != nullptr) && (datv_mod_report->isSet())){
        toJsonValue(QString("DATVModReport"), datv_mod_report, obj, QString("SWGDATVModReport"));
    }
    if((dsd_demod_report != nullptr) && (dsd_demod_report->isSet())){
        toJsonValue(QString("DSDDemodReport"), dsd_demod_report, obj, QString("SWGDSDDemodReport"));
    }
    if((ieee_802_15_4_mod_report != nullptr) && (ieee_802_15_4_mod_report->isSet())){
        toJsonValue(QString("IEEE_802_15_4_ModReport"), ieee_802_15_4_mod_report, obj, QString("SWGIEEE_802_15_4_ModReport"));
    }
    if((file_sink_report != nullptr) && (file_sink_report->isSet())){
        toJsonValue(QString("FileSinkReport"), file_sink_report, obj, QString("SWGFileSinkReport"));
    }
    if((file_source_report != nullptr) && (file_source_report->isSet())){
        toJsonValue(QString("FileSourceReport"), file_source_report, obj, QString("SWGFileSourceReport"));
    }
    if((free_dv_demod_report != nullptr) && (free_dv_demod_report->isSet())){
        toJsonValue(QString("FreeDVDemodReport"), free_dv_demod_report, obj, QString("SWGFreeDVDemodReport"));
    }
    if((free_dv_mod_report != nullptr) && (free_dv_mod_report->isSet())){
        toJsonValue(QString("FreeDVModReport"), free_dv_mod_report, obj, QString("SWGFreeDVModReport"));
    }
    if((freq_tracker_report != nullptr) && (freq_tracker_report->isSet())){
        toJsonValue(QString("FreqTrackerReport"), freq_tracker_report, obj, QString("SWGFreqTrackerReport"));
    }
    if((nfm_demod_report != nullptr) && (nfm_demod_report->isSet())){
        toJsonValue(QString("NFMDemodReport"), nfm_demod_report, obj, QString("SWGNFMDemodReport"));
    }
    if((nfm_mod_report != nullptr) && (nfm_mod_report->isSet())){
        toJsonValue(QString("NFMModReport"), nfm_mod_report, obj, QString("SWGNFMModReport"));
    }
    if((noise_figure_report != nullptr) && (noise_figure_report->isSet())){
        toJsonValue(QString("NoiseFigureReport"), noise_figure_report, obj, QString("SWGNoiseFigureReport"));
    }
    if((ssb_demod_report != nullptr) && (ssb_demod_report->isSet())){
        toJsonValue(QString("SSBDemodReport"), ssb_demod_report, obj, QString("SWGSSBDemodReport"));
    }
    if((radio_astronomy_report != nullptr) && (radio_astronomy_report->isSet())){
        toJsonValue(QString("RadioAstronomyReport"), radio_astronomy_report, obj, QString("SWGRadioAstronomyReport"));
    }
    if((radio_clock_report != nullptr) && (radio_clock_report->isSet())){
        toJsonValue(QString("RadioClockReport"), radio_clock_report, obj, QString("SWGRadioClockReport"));
    }
    if((remote_source_report != nullptr) && (remote_source_report->isSet())){
        toJsonValue(QString("RemoteSourceReport"), remote_source_report, obj, QString("SWGRemoteSourceReport"));
    }
    if((packet_demod_report != nullptr) && (packet_demod_report->isSet())){
        toJsonValue(QString("PacketDemodReport"), packet_demod_report, obj, QString("SWGPacketDemodReport"));
    }
    if((packet_mod_report != nullptr) && (packet_mod_report->isSet())){
        toJsonValue(QString("PacketModReport"), packet_mod_report, obj, QString("SWGPacketModReport"));
    }
    if((pager_demod_report != nullptr) && (pager_demod_report->isSet())){
        toJsonValue(QString("PagerDemodReport"), pager_demod_report, obj, QString("SWGPagerDemodReport"));
    }
    if((sig_mf_file_sink_report != nullptr) && (sig_mf_file_sink_report->isSet())){
        toJsonValue(QString("SigMFFileSinkReport"), sig_mf_file_sink_report, obj, QString("SWGSigMFFileSinkReport"));
    }
    if((ssb_mod_report != nullptr) && (ssb_mod_report->isSet())){
        toJsonValue(QString("SSBModReport"), ssb_mod_report, obj, QString("SWGSSBModReport"));
    }
    if((udp_source_report != nullptr) && (udp_source_report->isSet())){
        toJsonValue(QString("UDPSourceReport"), udp_source_report, obj, QString("SWGUDPSourceReport"));
    }
    if((udp_sink_report != nullptr) && (udp_sink_report->isSet())){
        toJsonValue(QString("UDPSinkReport"), udp_sink_report, obj, QString("SWGUDPSinkReport"));
    }
    if((vor_demod_report != nullptr) && (vor_demod_report->isSet())){
        toJsonValue(QString("VORDemodReport"), vor_demod_report, obj, QString("SWGVORDemodReport"));
    }
    if((vor_demod_sc_report != nullptr) && (vor_demod_sc_report->isSet())){
        toJsonValue(QString("VORDemodSCReport"), vor_demod_sc_report, obj, QString("SWGVORDemodSCReport"));
    }
    if((wfm_demod_report != nullptr) && (wfm_demod_report->isSet())){
        toJsonValue(QString("WFMDemodReport"), wfm_demod_report, obj, QString("SWGWFMDemodReport"));
    }
    if((wfm_mod_report != nullptr) && (wfm_mod_report->isSet())){
        toJsonValue(QString("WFMModReport"), wfm_mod_report, obj, QString("SWGWFMModReport"));
    }

    return obj;
}

QString*
SWGChannelReport::getChannelType() {
    return channel_type;
}
void
SWGChannelReport::setChannelType(QString* channel_type) {
    this->channel_type = channel_type;
    this->m_channel_type_isSet = true;
}

qint32
SWGChannelReport::getDirection() {
    return direction;
}
void
SWGChannelReport::setDirection(qint32 direction) {
    this->direction = direction;
    this->m_direction_isSet = true;
}

SWGADSBDemodReport*
SWGChannelReport::getAdsbDemodReport() {
    return adsb_demod_report;
}
void
SWGChannelReport::setAdsbDemodReport(SWGADSBDemodReport* adsb_demod_report) {
    this->adsb_demod_report = adsb_demod_report;
    this->m_adsb_demod_report_isSet = true;
}

SWGAISDemodReport*
SWGChannelReport::getAisDemodReport() {
    return ais_demod_report;
}
void
SWGChannelReport::setAisDemodReport(SWGAISDemodReport* ais_demod_report) {
    this->ais_demod_report = ais_demod_report;
    this->m_ais_demod_report_isSet = true;
}

SWGAISModReport*
SWGChannelReport::getAisModReport() {
    return ais_mod_report;
}
void
SWGChannelReport::setAisModReport(SWGAISModReport* ais_mod_report) {
    this->ais_mod_report = ais_mod_report;
    this->m_ais_mod_report_isSet = true;
}

SWGAMDemodReport*
SWGChannelReport::getAmDemodReport() {
    return am_demod_report;
}
void
SWGChannelReport::setAmDemodReport(SWGAMDemodReport* am_demod_report) {
    this->am_demod_report = am_demod_report;
    this->m_am_demod_report_isSet = true;
}

SWGAMModReport*
SWGChannelReport::getAmModReport() {
    return am_mod_report;
}
void
SWGChannelReport::setAmModReport(SWGAMModReport* am_mod_report) {
    this->am_mod_report = am_mod_report;
    this->m_am_mod_report_isSet = true;
}

SWGATVModReport*
SWGChannelReport::getAtvModReport() {
    return atv_mod_report;
}
void
SWGChannelReport::setAtvModReport(SWGATVModReport* atv_mod_report) {
    this->atv_mod_report = atv_mod_report;
    this->m_atv_mod_report_isSet = true;
}

SWGBFMDemodReport*
SWGChannelReport::getBfmDemodReport() {
    return bfm_demod_report;
}
void
SWGChannelReport::setBfmDemodReport(SWGBFMDemodReport* bfm_demod_report) {
    this->bfm_demod_report = bfm_demod_report;
    this->m_bfm_demod_report_isSet = true;
}

SWGChirpChatDemodReport*
SWGChannelReport::getChirpChatDemodReport() {
    return chirp_chat_demod_report;
}
void
SWGChannelReport::setChirpChatDemodReport(SWGChirpChatDemodReport* chirp_chat_demod_report) {
    this->chirp_chat_demod_report = chirp_chat_demod_report;
    this->m_chirp_chat_demod_report_isSet = true;
}

SWGChirpChatModReport*
SWGChannelReport::getChirpChatModReport() {
    return chirp_chat_mod_report;
}
void
SWGChannelReport::setChirpChatModReport(SWGChirpChatModReport* chirp_chat_mod_report) {
    this->chirp_chat_mod_report = chirp_chat_mod_report;
    this->m_chirp_chat_mod_report_isSet = true;
}

SWGDATVDemodReport*
SWGChannelReport::getDatvDemodReport() {
    return datv_demod_report;
}
void
SWGChannelReport::setDatvDemodReport(SWGDATVDemodReport* datv_demod_report) {
    this->datv_demod_report = datv_demod_report;
    this->m_datv_demod_report_isSet = true;
}

SWGDATVModReport*
SWGChannelReport::getDatvModReport() {
    return datv_mod_report;
}
void
SWGChannelReport::setDatvModReport(SWGDATVModReport* datv_mod_report) {
    this->datv_mod_report = datv_mod_report;
    this->m_datv_mod_report_isSet = true;
}

SWGDSDDemodReport*
SWGChannelReport::getDsdDemodReport() {
    return dsd_demod_report;
}
void
SWGChannelReport::setDsdDemodReport(SWGDSDDemodReport* dsd_demod_report) {
    this->dsd_demod_report = dsd_demod_report;
    this->m_dsd_demod_report_isSet = true;
}

SWGIEEE_802_15_4_ModReport*
SWGChannelReport::getIeee802154ModReport() {
    return ieee_802_15_4_mod_report;
}
void
SWGChannelReport::setIeee802154ModReport(SWGIEEE_802_15_4_ModReport* ieee_802_15_4_mod_report) {
    this->ieee_802_15_4_mod_report = ieee_802_15_4_mod_report;
    this->m_ieee_802_15_4_mod_report_isSet = true;
}

SWGFileSinkReport*
SWGChannelReport::getFileSinkReport() {
    return file_sink_report;
}
void
SWGChannelReport::setFileSinkReport(SWGFileSinkReport* file_sink_report) {
    this->file_sink_report = file_sink_report;
    this->m_file_sink_report_isSet = true;
}

SWGFileSourceReport*
SWGChannelReport::getFileSourceReport() {
    return file_source_report;
}
void
SWGChannelReport::setFileSourceReport(SWGFileSourceReport* file_source_report) {
    this->file_source_report = file_source_report;
    this->m_file_source_report_isSet = true;
}

SWGFreeDVDemodReport*
SWGChannelReport::getFreeDvDemodReport() {
    return free_dv_demod_report;
}
void
SWGChannelReport::setFreeDvDemodReport(SWGFreeDVDemodReport* free_dv_demod_report) {
    this->free_dv_demod_report = free_dv_demod_report;
    this->m_free_dv_demod_report_isSet = true;
}

SWGFreeDVModReport*
SWGChannelReport::getFreeDvModReport() {
    return free_dv_mod_report;
}
void
SWGChannelReport::setFreeDvModReport(SWGFreeDVModReport* free_dv_mod_report) {
    this->free_dv_mod_report = free_dv_mod_report;
    this->m_free_dv_mod_report_isSet = true;
}

SWGFreqTrackerReport*
SWGChannelReport::getFreqTrackerReport() {
    return freq_tracker_report;
}
void
SWGChannelReport::setFreqTrackerReport(SWGFreqTrackerReport* freq_tracker_report) {
    this->freq_tracker_report = freq_tracker_report;
    this->m_freq_tracker_report_isSet = true;
}

SWGNFMDemodReport*
SWGChannelReport::getNfmDemodReport() {
    return nfm_demod_report;
}
void
SWGChannelReport::setNfmDemodReport(SWGNFMDemodReport* nfm_demod_report) {
    this->nfm_demod_report = nfm_demod_report;
    this->m_nfm_demod_report_isSet = true;
}

SWGNFMModReport*
SWGChannelReport::getNfmModReport() {
    return nfm_mod_report;
}
void
SWGChannelReport::setNfmModReport(SWGNFMModReport* nfm_mod_report) {
    this->nfm_mod_report = nfm_mod_report;
    this->m_nfm_mod_report_isSet = true;
}

SWGNoiseFigureReport*
SWGChannelReport::getNoiseFigureReport() {
    return noise_figure_report;
}
void
SWGChannelReport::setNoiseFigureReport(SWGNoiseFigureReport* noise_figure_report) {
    this->noise_figure_report = noise_figure_report;
    this->m_noise_figure_report_isSet = true;
}

SWGSSBDemodReport*
SWGChannelReport::getSsbDemodReport() {
    return ssb_demod_report;
}
void
SWGChannelReport::setSsbDemodReport(SWGSSBDemodReport* ssb_demod_report) {
    this->ssb_demod_report = ssb_demod_report;
    this->m_ssb_demod_report_isSet = true;
}

SWGRadioAstronomyReport*
SWGChannelReport::getRadioAstronomyReport() {
    return radio_astronomy_report;
}
void
SWGChannelReport::setRadioAstronomyReport(SWGRadioAstronomyReport* radio_astronomy_report) {
    this->radio_astronomy_report = radio_astronomy_report;
    this->m_radio_astronomy_report_isSet = true;
}

SWGRadioClockReport*
SWGChannelReport::getRadioClockReport() {
    return radio_clock_report;
}
void
SWGChannelReport::setRadioClockReport(SWGRadioClockReport* radio_clock_report) {
    this->radio_clock_report = radio_clock_report;
    this->m_radio_clock_report_isSet = true;
}

SWGRemoteSourceReport*
SWGChannelReport::getRemoteSourceReport() {
    return remote_source_report;
}
void
SWGChannelReport::setRemoteSourceReport(SWGRemoteSourceReport* remote_source_report) {
    this->remote_source_report = remote_source_report;
    this->m_remote_source_report_isSet = true;
}

SWGPacketDemodReport*
SWGChannelReport::getPacketDemodReport() {
    return packet_demod_report;
}
void
SWGChannelReport::setPacketDemodReport(SWGPacketDemodReport* packet_demod_report) {
    this->packet_demod_report = packet_demod_report;
    this->m_packet_demod_report_isSet = true;
}

SWGPacketModReport*
SWGChannelReport::getPacketModReport() {
    return packet_mod_report;
}
void
SWGChannelReport::setPacketModReport(SWGPacketModReport* packet_mod_report) {
    this->packet_mod_report = packet_mod_report;
    this->m_packet_mod_report_isSet = true;
}

SWGPagerDemodReport*
SWGChannelReport::getPagerDemodReport() {
    return pager_demod_report;
}
void
SWGChannelReport::setPagerDemodReport(SWGPagerDemodReport* pager_demod_report) {
    this->pager_demod_report = pager_demod_report;
    this->m_pager_demod_report_isSet = true;
}

SWGSigMFFileSinkReport*
SWGChannelReport::getSigMfFileSinkReport() {
    return sig_mf_file_sink_report;
}
void
SWGChannelReport::setSigMfFileSinkReport(SWGSigMFFileSinkReport* sig_mf_file_sink_report) {
    this->sig_mf_file_sink_report = sig_mf_file_sink_report;
    this->m_sig_mf_file_sink_report_isSet = true;
}

SWGSSBModReport*
SWGChannelReport::getSsbModReport() {
    return ssb_mod_report;
}
void
SWGChannelReport::setSsbModReport(SWGSSBModReport* ssb_mod_report) {
    this->ssb_mod_report = ssb_mod_report;
    this->m_ssb_mod_report_isSet = true;
}

SWGUDPSourceReport*
SWGChannelReport::getUdpSourceReport() {
    return udp_source_report;
}
void
SWGChannelReport::setUdpSourceReport(SWGUDPSourceReport* udp_source_report) {
    this->udp_source_report = udp_source_report;
    this->m_udp_source_report_isSet = true;
}

SWGUDPSinkReport*
SWGChannelReport::getUdpSinkReport() {
    return udp_sink_report;
}
void
SWGChannelReport::setUdpSinkReport(SWGUDPSinkReport* udp_sink_report) {
    this->udp_sink_report = udp_sink_report;
    this->m_udp_sink_report_isSet = true;
}

SWGVORDemodReport*
SWGChannelReport::getVorDemodReport() {
    return vor_demod_report;
}
void
SWGChannelReport::setVorDemodReport(SWGVORDemodReport* vor_demod_report) {
    this->vor_demod_report = vor_demod_report;
    this->m_vor_demod_report_isSet = true;
}

SWGVORDemodSCReport*
SWGChannelReport::getVorDemodScReport() {
    return vor_demod_sc_report;
}
void
SWGChannelReport::setVorDemodScReport(SWGVORDemodSCReport* vor_demod_sc_report) {
    this->vor_demod_sc_report = vor_demod_sc_report;
    this->m_vor_demod_sc_report_isSet = true;
}

SWGWFMDemodReport*
SWGChannelReport::getWfmDemodReport() {
    return wfm_demod_report;
}
void
SWGChannelReport::setWfmDemodReport(SWGWFMDemodReport* wfm_demod_report) {
    this->wfm_demod_report = wfm_demod_report;
    this->m_wfm_demod_report_isSet = true;
}

SWGWFMModReport*
SWGChannelReport::getWfmModReport() {
    return wfm_mod_report;
}
void
SWGChannelReport::setWfmModReport(SWGWFMModReport* wfm_mod_report) {
    this->wfm_mod_report = wfm_mod_report;
    this->m_wfm_mod_report_isSet = true;
}


bool
SWGChannelReport::isSet(){
    bool isObjectUpdated = false;
    do{
        if(channel_type && *channel_type != QString("")){
            isObjectUpdated = true; break;
        }
        if(m_direction_isSet){
            isObjectUpdated = true; break;
        }
        if(adsb_demod_report && adsb_demod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(ais_demod_report && ais_demod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(ais_mod_report && ais_mod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(am_demod_report && am_demod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(am_mod_report && am_mod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(atv_mod_report && atv_mod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(bfm_demod_report && bfm_demod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(chirp_chat_demod_report && chirp_chat_demod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(chirp_chat_mod_report && chirp_chat_mod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(datv_demod_report && datv_demod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(datv_mod_report && datv_mod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(dsd_demod_report && dsd_demod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(ieee_802_15_4_mod_report && ieee_802_15_4_mod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(file_sink_report && file_sink_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(file_source_report && file_source_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(free_dv_demod_report && free_dv_demod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(free_dv_mod_report && free_dv_mod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(freq_tracker_report && freq_tracker_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(nfm_demod_report && nfm_demod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(nfm_mod_report && nfm_mod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(noise_figure_report && noise_figure_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(ssb_demod_report && ssb_demod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(radio_astronomy_report && radio_astronomy_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(radio_clock_report && radio_clock_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(remote_source_report && remote_source_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(packet_demod_report && packet_demod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(packet_mod_report && packet_mod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(pager_demod_report && pager_demod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(sig_mf_file_sink_report && sig_mf_file_sink_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(ssb_mod_report && ssb_mod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(udp_source_report && udp_source_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(udp_sink_report && udp_sink_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(vor_demod_report && vor_demod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(vor_demod_sc_report && vor_demod_sc_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(wfm_demod_report && wfm_demod_report->isSet()){
            isObjectUpdated = true; break;
        }
        if(wfm_mod_report && wfm_mod_report->isSet()){
            isObjectUpdated = true; break;
        }
    }while(false);
    return isObjectUpdated;
}
}

