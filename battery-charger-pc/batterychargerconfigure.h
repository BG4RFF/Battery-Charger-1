/*
Title:    Atmel Microcontroller Battery Charger Project
*/

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

#ifndef BATTERYCHARGERCONFIGURE_H
#define BATTERYCHARGERCONFIGURE_H

#include "ui_batterychargerconfiguredialogue.h"
#include "acqunit.h"
#include <QString>

//-----------------------------------------------------------------------------
/** @brief Acquisition Configure Window.

This class provides a window with a number of items for basic configuration.
*/

class BatteryChargerConfigure : public QDialog
{
    Q_OBJECT
public:
    BatteryChargerConfigure(AcqUnit*, QWidget* parent = 0);
    ~BatteryChargerConfigure();
private slots:
    void on_closeButton_clicked();
    void on_batteryCapacityCheckBox_stateChanged (int state);
    void on_advancedButton_clicked();
    void on_unitConfigButton_clicked();
    void on_updateButton_clicked();
    void displayErrorMessage(const QString);
private:
    AcqUnit* unit;
    AcqAppConfigUnit* appConfigureUnit;
    float batteryCapacity;
    float floatChargeVoltageThreshold;
    float cyclicChargeCurrentThreshold;
    float cyclicChargeVoltageThreshold;
    float cyclicChargeCurrentCutoff;
    Ui::BatteryChargerConfigureDialogue BatteryChargerConfigureDialogUi;// User Interface
};

//-----------------------------------------------------------------------------
#endif
