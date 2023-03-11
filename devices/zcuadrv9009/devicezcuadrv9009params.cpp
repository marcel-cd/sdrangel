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

#include "devicezcuadrv9009params.h"

#include "devicezcuadrv9009.h"

DeviceZcuadrv9009Params::DeviceZcuadrv9009Params() :
    m_box(nullptr)
{
}

DeviceZcuadrv9009Params::~DeviceZcuadrv9009Params()
{
}

bool DeviceZcuadrv9009Params::open(const std::string& serial)
{
    m_box = DeviceZcuadrv9009::instance().getDeviceFromSerial(serial);
    return m_box != nullptr;
}

bool DeviceZcuadrv9009Params::openURI(const std::string& uri)
{
    m_box = DeviceZcuadrv9009::instance().getDeviceFromURI(uri);
    return m_box != nullptr;
}

void DeviceZcuadrv9009Params::close()
{
    delete m_box;
    m_box = nullptr;
}
