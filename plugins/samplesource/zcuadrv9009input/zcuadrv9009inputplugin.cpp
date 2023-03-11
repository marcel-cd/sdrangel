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

#include <QtPlugin>

#include "plugin/pluginapi.h"
#include "zcuadrv9009/devicezcuadrv9009.h"

#ifdef SERVER_MODE
#include "zcuadrv9009input.h"
#else
#include "zcuadrv9009inputgui.h"
#endif
#include "zcuadrv9009inputplugin.h"
#include "zcuadrv9009inputwebapiadapter.h"

class DeviceAPI;

const PluginDescriptor Zcuadrv9009InputPlugin::m_pluginDescriptor = {
    QStringLiteral("Zcuadrv9009"),
	QStringLiteral("Zcuadrv9009 Input"),
    QStringLiteral("7.8.2"),
	QStringLiteral("(c) Edouard Griffiths, F4EXB"),
	QStringLiteral("https://github.com/f4exb/sdrangel"),
	true,
	QStringLiteral("https://github.com/f4exb/sdrangel")
};

static constexpr const char* const m_hardwareID = "Zcuadrv9009";
const char* const Zcuadrv9009InputPlugin::m_deviceTypeID = ZCUADRV9009_DEVICE_TYPE_ID;

Zcuadrv9009InputPlugin::Zcuadrv9009InputPlugin(QObject* parent) :
	QObject(parent)
{
}

const PluginDescriptor& Zcuadrv9009InputPlugin::getPluginDescriptor() const
{
	return m_pluginDescriptor;
}

void Zcuadrv9009InputPlugin::initPlugin(PluginAPI* pluginAPI)
{
	pluginAPI->registerSampleSource(m_deviceTypeID, this);
	DeviceZcuadrv9009::instance(); // create singleton
}

void Zcuadrv9009InputPlugin::enumOriginDevices(QStringList& listedHwIds, OriginDevices& originDevices)
{
    if (listedHwIds.contains(m_hardwareID)) { // check if it was done
        return;
    }

    DeviceZcuadrv9009::instance().enumOriginDevices(m_hardwareID, originDevices);
    listedHwIds.append(m_hardwareID);
}

PluginInterface::SamplingDevices Zcuadrv9009InputPlugin::enumSampleSources(const OriginDevices& originDevices)
{
	SamplingDevices result;

	for (OriginDevices::const_iterator it = originDevices.begin(); it != originDevices.end(); ++it)
    {
        if (it->hardwareId == m_hardwareID)
        {
            result.append(SamplingDevice(
                it->displayableName,
                it->hardwareId,
                m_deviceTypeID,
                it->serial,
                it->sequence,
                PluginInterface::SamplingDevice::PhysicalDevice,
                PluginInterface::SamplingDevice::StreamSingleRx,
                1,
                0
            ));
            qDebug("Zcuadrv9009InputPlugin::enumSampleSources: enumerated Zcuadrv9009 device #%d", it->sequence);
        }
    }

	return result;
}

#ifdef SERVER_MODE
DeviceGUI* Zcuadrv9009InputPlugin::createSampleSourcePluginInstanceGUI(
        const QString& sourceId,
        QWidget **widget,
        DeviceUISet *deviceUISet)
{
    (void) sourceId;
    (void) widget;
    (void) deviceUISet;
    return 0;
}
#else
DeviceGUI* Zcuadrv9009InputPlugin::createSampleSourcePluginInstanceGUI(
        const QString& sourceId,
        QWidget **widget,
        DeviceUISet *deviceUISet)
{
	if(sourceId == m_deviceTypeID)
	{
		Zcuadrv9009InputGui* gui = new Zcuadrv9009InputGui(deviceUISet);
		*widget = gui;
		return gui;
	}
	else
	{
		return 0;
	}
}
#endif

DeviceSampleSource *Zcuadrv9009InputPlugin::createSampleSourcePluginInstance(const QString& sourceId, DeviceAPI *deviceAPI)
{
    if (sourceId == m_deviceTypeID)
    {
        Zcuadrv9009Input* input = new Zcuadrv9009Input(deviceAPI);
        return input;
    }
    else
    {
        return 0;
    }
}

DeviceWebAPIAdapter *Zcuadrv9009InputPlugin::createDeviceWebAPIAdapter() const
{
    return new Zcuadrv9009InputWebAPIAdapter();
}
