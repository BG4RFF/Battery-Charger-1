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

#ifndef BATTERYCHARGERCALIBRATE_H
#define BATTERYCHARGERCALIBRATE_H

#include "ui_batterychargercalibratedialogue.h"
#include "acqsimpleadcunit.h"
#include <QString>

//-----------------------------------------------------------------------------
/** @brief Acquisition Channel Calibration Window.

This class provides a window for entering measured quantities associated with the
channel, and calculating from these the scale factors.
*/

class BatteryChargerCalibrate : public QDialog
{
    Q_OBJECT
public:
    BatteryChargerCalibrate(AcqSimpleAdcUnit*, QWidget* parent = 0);
    ~BatteryChargerCalibrate();
private slots:
    void on_closeButton_clicked();
    void on_firstValueLineEdit_textChanged();
    void on_secondValueLineEdit_textChanged();
    void on_firstValueSetButton_clicked();
    void on_secondValueSetButton_clicked();
    void on_commitButton_clicked();
    void on_clearEntriesButton_clicked();
    void displayErrorMessage(const QString);
private:
    AcqSimpleAdcUnit* channelUnit;
    float firstValue;
    float secondValue;
    int firstSample;
    int secondSample;
    bool firstEntered;
    Ui::BatteryChargerCalibrateDialogue BatteryChargerCalibrateDialogueUi;// User Interface
};

//-----------------------------------------------------------------------------
#endif
