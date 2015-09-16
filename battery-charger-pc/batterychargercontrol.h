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

#ifndef BATTERYCHARGERCONTROL_H
#define BATTERYCHARGERCONTROL_H

#include "ui_batterychargercontroldialogue.h"
#include "acqsimpleadcunit.h"
#include "serialport.h"
#include <QDir>
#include <QFile>
#include <QDateTime>
#include <QString>

//-----------------------------------------------------------------------------
/** @brief Acquisition Control Window.

This class provides an initial window that appears when a properly configured
Data Acquisition Unit has been synchronized. The window is configured
according to the detected characteristics of the device and some basic
controls and information are provided. From these, suitable windows may be
launched to provide detailed control of the device channels.
@todo This class contains code that is specific to the A/D converter module.
This really should be removed to purify the class. A possible way could be to
use the plugin features of Qt so that code relevant to the channel type can be
separated out and only added in when necessary.
*/

class BatteryChargerControl : public QDialog
{
    Q_OBJECT
public:
    BatteryChargerControl(AcqUnit*, QWidget* parent = 0);
    ~BatteryChargerControl();
    bool success();
    QString error();
private slots:
    void on_closeButton_clicked();
    void on_uploadButton_clicked();
    void on_configureButton_clicked();
//    void on_offlineCheckBox_stateChanged(int);
    void on_closeFileButton_clicked();
    void on_downloadButton_clicked();
    void on_acquireButton_clicked();
    void on_saveFileButton_clicked();
    void on_aboutButton_clicked();
private:
    void displayErrorMessage(const QString message);
    SerialPort* port;
    bool synchronized;
    QString errorMessage;
    QDir saveDirectory;
    QString saveFile;
    QFile* outFile;
    QDateTime startTime;
    double standardDeviation;
    double averageValue;
    unsigned int runTime;
    unsigned int interval;
    AcqUnit* unit;
    Ui::BatteryChargerControlDialogue solarRegulatorControlDialogUi;// User Interface
};

//-----------------------------------------------------------------------------
#endif
