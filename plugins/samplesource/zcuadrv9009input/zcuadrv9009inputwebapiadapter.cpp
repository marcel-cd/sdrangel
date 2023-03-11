///////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2019 Edouard Griffiths, F4EXB                                   //
//                                                                               //
// Implementation of static web API adapters used for preset serialization and   //
// deserialization                                                               //
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

#include "SWGDeviceSettings.h"
#include "zcuadrv9009input.h"
#include "zcuadrv9009inputwebapiadapter.h"

Zcuadrv9009InputWebAPIAdapter::Zcuadrv9009InputWebAPIAdapter()
{}

Zcuadrv9009InputWebAPIAdapter::~Zcuadrv9009InputWebAPIAdapter()
{}

int Zcuadrv9009InputWebAPIAdapter::webapiSettingsGet(
        SWGSDRangel::SWGDeviceSettings& response,
        QString& errorMessage)
{
    (void) errorMessage;
    response.setZcuadrv9009InputSettings(new SWGSDRangel::SWGZcuadrv9009InputSettings());
    response.getZcuadrv9009InputSettings()->init();
    Zcuadrv9009Input::webapiFormatDeviceSettings(response, m_settings);
    return 200;
}

int Zcuadrv9009InputWebAPIAdapter::webapiSettingsPutPatch(
        bool force,
        const QStringList& deviceSettingsKeys,
        SWGSDRangel::SWGDeviceSettings& response, // query + response
        QString& errorMessage)
{
    (void) force; // no action
    (void) errorMessage;
    Zcuadrv9009Input::webapiUpdateDeviceSettings(m_settings, deviceSettingsKeys, response);
    return 200;
}
