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

#include "devicezcuadrv9009.h"

const uint64_t DeviceZcuadrv9009::rxLOLowLimitFreq  =   70000000UL; // 70 MHz: take AD9364 specs
const uint64_t DeviceZcuadrv9009::rxLOHighLimitFreq = 6000000000UL; //  6 GHz: take AD9364 specs

const uint64_t DeviceZcuadrv9009::txLOLowLimitFreq  =   46875000UL; // 46.875 MHz: take AD9364 specs
const uint64_t DeviceZcuadrv9009::txLOHighLimitFreq = 6000000000UL; //  6 GHz: take AD9364 specs

const uint32_t DeviceZcuadrv9009::srLowLimitFreq  = (25000000U/12U)+3U; // 25/12 MS/s without FIR interpolation/decimation (+3 so it is the next multiple of 4)
const uint32_t DeviceZcuadrv9009::srHighLimitFreq = 20000000U;     // 20 MS/s: take AD9363 speces

const uint32_t DeviceZcuadrv9009::bbLPRxLowLimitFreq  =   200000U; // 200 kHz
const uint32_t DeviceZcuadrv9009::bbLPRxHighLimitFreq = 14000000U; // 14 MHz
const uint32_t DeviceZcuadrv9009::bbLPTxLowLimitFreq  =   625000U; // 625 kHz
const uint32_t DeviceZcuadrv9009::bbLPTxHighLimitFreq = 16000000U; // 16 MHz

const float DeviceZcuadrv9009::firBWLowLimitFactor  = 0.05f;
const float DeviceZcuadrv9009::firBWHighLimitFactor = 0.9f;

DeviceZcuadrv9009::DeviceZcuadrv9009()
{
}

DeviceZcuadrv9009::~DeviceZcuadrv9009()
{
}

DeviceZcuadrv9009& DeviceZcuadrv9009::instance()
{
    static DeviceZcuadrv9009 inst;
    return inst;
}

DeviceZcuadrv9009Box* DeviceZcuadrv9009::getDeviceFromURI(const std::string& uri)
{
    return new DeviceZcuadrv9009Box(uri);
}

DeviceZcuadrv9009Box* DeviceZcuadrv9009::getDeviceFromSerial(const std::string& serial)
{
    const std::string *uri = m_scan.getURIFromSerial(serial);

    if (uri) {
        return new DeviceZcuadrv9009Box(*uri);
    } else {
        return 0;
    }
}




