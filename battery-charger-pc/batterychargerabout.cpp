/*
Title:    Atmel Microcontroller Battery Charger Project
*/
/** @defgroup batterychargerabout Battery Charger Information Window

These functions provide functionality for the Battery Charger Information
window that provides basic information about the device and the channels, as
well as copyright and version.

@{*/
/***************************************************************************
 *   Copyright (C) 2010 by Ken Sarkies                                     *
 *   ksarkies@trinity.asn.au ksarkies@internode.on.net                     *
 *                                                                         *
 *   This file is part of Acquisition.                                     *
 *                                                                         *
 *   Battery Charger is free software; you can redistribute it and/or      *
 *   modify it under the terms of the GNU General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   Battery Charger is distributed in the hope that it will be useful,    *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with Battery Charger. If not, write to the                      *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA.             *
 ***************************************************************************/

#include "batterychargerabout.h"

//-----------------------------------------------------------------------------
/** @brief Constructor.

@param u Acquisition Unit Object
@param parent Parent widget.
*/
BatteryChargerAbout::BatteryChargerAbout(AcqUnit* u,
          QWidget* parent) : QDialog(parent)
{
    unit = u;
//! Build the User Interface display from Ui object.
    BatteryChargerAboutDialogUi.setupUi(this);
    setupGUI();
}

BatteryChargerAbout::~BatteryChargerAbout()
{
}

//-----------------------------------------------------------------------------
/** @brief Close when OK is activated returning success.
*/

void BatteryChargerAbout::on_closeButton_clicked()
{
    accept();
}

//-----------------------------------------------------------------------------
/** @brief Setup the GUI from the unit characteristics.

Interpret the Information Block, and setup the GUI to show appropriate
controls.
*/

void BatteryChargerAbout::setupGUI()
{
    BatteryChargerAboutDialogUi.boardIDLabel->
                              setText(QString("Board ID: %1")
                              .arg(unit->getBoardID(),2,16,QLatin1Char('0')));
    BatteryChargerAboutDialogUi.cpuFreqLabel->
                              setText(QString("CPU Frequency: %1 MHz")
                              .arg((int)(unit->getCpuFreq()/1000),2,10));
    BatteryChargerAboutDialogUi.cpuTypeLabel->
                              setText(QString("CPU Type: %1")
                              .arg(unit->getMCUDescriptor()));
    unsigned int numberChannels = unit->getNumberChannels();
    BatteryChargerAboutDialogUi.channelALabel->
                          setText(QString("Battery Voltage Channel Not Configured"));
    BatteryChargerAboutDialogUi.channelBLabel->
                          setText(QString("Load Current Channel Not Configured"));
    BatteryChargerAboutDialogUi.channelCLabel->
                          setText(QString("Panel Voltage Channel Not Configured"));
    BatteryChargerAboutDialogUi.channelDLabel->
                          setText(QString("Panel Current Channel Not Configured"));
    for (uint channel = 0; channel < numberChannels; channel++)
    {
        AcqSimpleAdcUnit* channelAUnit = new AcqSimpleAdcUnit(unit,channel);
        BatteryChargerAboutDialogUi.channelALabel->
                              setText(QString("Battery Voltage Precision: %1")
                              .arg(channelAUnit->getAdcPrecision()));
        AcqSimpleAdcUnit* channelBUnit = new AcqSimpleAdcUnit(unit,channel);
        BatteryChargerAboutDialogUi.channelBLabel->
                              setText(QString("Load Current Precision: %1")
                              .arg(channelBUnit->getAdcPrecision()));
        AcqSimpleAdcUnit* channelCUnit = new AcqSimpleAdcUnit(unit,channel);
        BatteryChargerAboutDialogUi.channelCLabel->
                              setText(QString("Panel Voltage Precision: %1")
                              .arg(channelCUnit->getAdcPrecision()));
        AcqSimpleAdcUnit* channelDUnit = new AcqSimpleAdcUnit(unit,channel);
        BatteryChargerAboutDialogUi.channelDLabel->
                              setText(QString("Panel Current Precision: %1")
                              .arg(channelDUnit->getAdcPrecision()));
    }
}
/**@}*/
//-----------------------------------------------------------------------------

