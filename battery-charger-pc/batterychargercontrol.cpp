/*
Title:    Atmel Microcontroller Battery Charger Project
*/
/** @defgroup batterychargercontrol Battery Charger Control Window

These functions provide functionality for the Battery Charger Control
Window in which the main control and data acquisition is done. This window
allows view of the current state of the charger internal variables,
retrieval of stored status data and access to a number of deeper configuration
dialogues.

14/2/2010 Changed the method of retrieving the channel type to match changes
to the acqunit module.
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

#include "batterychargercontrol.h"
#include "batterychargerconfigure.h"
#include "batterychargerabout.h"
#include "avrserialprog.h"
#include "serialport.h"
#include "acqpacket.h"
#include "acqgeneral.h"
#include <QApplication>
#include <QString>
#include <QLineEdit>
#include <QLabel>
#include <QMessageBox>
#include <QTextEdit>
#include <QCloseEvent>
#include <QDebug>
#include <QTime>
#include <QTimer>
#include <QFileDialog>
#include <QDir>
#include <cstdlib>
#include <iostream>
#include <cmath>

//-----------------------------------------------------------------------------
/** @brief Constructor.

@param u Acquisition Unit Object
@param parent Parent widget.
*/

BatteryChargerControl::BatteryChargerControl(AcqUnit* u,QWidget* parent)
                                           : QDialog(parent)
{
    unit = u;
//! Build the User Interface display from Ui object.
    solarRegulatorControlDialogUi.setupUi(this);
    saveFile = "";
// Disable a bunch of controls until the save file has been opened.
    solarRegulatorControlDialogUi.downloadButton->setEnabled(false);
    solarRegulatorControlDialogUi.closeFileButton->setEnabled(false);
    on_acquireButton_clicked();
}

BatteryChargerControl::~BatteryChargerControl()
{
}

//-----------------------------------------------------------------------------
/** @brief Close when quit is activated.
*/

void BatteryChargerControl::on_closeButton_clicked()
{
    accept();
}

//-----------------------------------------------------------------------------
/** @brief Read the A/D converter information.

This function interrogates the unit to obtain the measured values of voltages
and currents that it is currently using, as well as internal state variables
of interest.

The instantaneous values are stored in the internal EEPROM in the channel
information block.
*/

void BatteryChargerControl::on_acquireButton_clicked()
{
    solarRegulatorControlDialogUi.errorLabel->clear();
    unsigned char stateVariables[20];
    unsigned char numberBytes;
    unsigned char numberChannels = unit->getNumberChannels();
    unsigned char precision = 2;
    bool ok = unit->getStateVariables(0,stateVariables,numberBytes);
    AcqSimpleAdcUnit* panelVoltageUnit = new AcqSimpleAdcUnit(unit,0);
// Pull in a value for the panel voltage
    int panelVoltageSample = stateVariables[0]+(stateVariables[1] << 8);
    if (panelVoltageSample > 32768) panelVoltageSample -= 65536;
    unsigned char adcPrecision  = panelVoltageUnit->getAdcPrecision();
    float mulScaleFactor = panelVoltageUnit->getMulScaleFactor();
    float offScaleFactor = panelVoltageUnit->getOffScaleFactor();
    float panelVoltage = ((panelVoltageSample*mulScaleFactor)/(1<<adcPrecision)
                            +offScaleFactor)/655.36;
    AcqSimpleAdcUnit* batteryVoltageUnit = new AcqSimpleAdcUnit(unit,1);
// Pull in a value for the battery voltage
    int batteryVoltageSample = stateVariables[2]+(stateVariables[3] << 8);
    if (batteryVoltageSample > 32768) batteryVoltageSample -= 65536;
    adcPrecision  = batteryVoltageUnit->getAdcPrecision();
    mulScaleFactor = batteryVoltageUnit->getMulScaleFactor();
    offScaleFactor = batteryVoltageUnit->getOffScaleFactor();
    float batteryVoltage = ((batteryVoltageSample*mulScaleFactor)/(1<<adcPrecision)
                            +offScaleFactor)/655.36;
    AcqSimpleAdcUnit* loadCurrentUnit = new AcqSimpleAdcUnit(unit,2);
// Pull in a value for the load current
    int loadCurrentSample = stateVariables[4]+(stateVariables[5] << 8);
    if (loadCurrentSample > 32768) loadCurrentSample -= 65536;
    adcPrecision  = loadCurrentUnit->getAdcPrecision();
    mulScaleFactor = loadCurrentUnit->getMulScaleFactor();
    offScaleFactor = loadCurrentUnit->getOffScaleFactor();
    float loadCurrent = ((loadCurrentSample*mulScaleFactor)/(1<<adcPrecision)
                            +offScaleFactor)/655.36;
    AcqSimpleAdcUnit* panelCurrentUnit = new AcqSimpleAdcUnit(unit,3);
// Pull in a value for the panel current
    int panelCurrentSample = stateVariables[6]+(stateVariables[7] << 8);
    if (panelCurrentSample > 32768) panelCurrentSample -= 65536;
    adcPrecision  = panelCurrentUnit->getAdcPrecision();
    mulScaleFactor = panelCurrentUnit->getMulScaleFactor();
    offScaleFactor = panelCurrentUnit->getOffScaleFactor();
    float panelCurrent = ((panelCurrentSample*mulScaleFactor)/(1<<adcPrecision)
                            +offScaleFactor)/655.36;
// Get the averaged battery current and voltage
    int averageBatteryVoltageSample = stateVariables[8]+(stateVariables[9] << 8);
    float averageBatteryVoltage = averageBatteryVoltageSample/655.36;
    int averageBatteryCurrentSample = stateVariables[10]+(stateVariables[11] << 8);
    float averageBatteryCurrent = averageBatteryCurrentSample/655.36;
    int scaledPanelCurrentSample = stateVariables[12]+(stateVariables[13] << 8);
qDebug() << panelCurrentSample << panelCurrent << averageBatteryCurrentSample << averageBatteryCurrent << scaledPanelCurrentSample;
// Now display the results if possible
    if (panelVoltageUnit->isConfigured())
    {
        solarRegulatorControlDialogUi.panelVoltageDisplay->setVisible(true);
        if (ok)
        {
            solarRegulatorControlDialogUi.panelVoltageDisplay->display(QString("%1")
                          .arg(panelVoltage,5,'f',precision));
        }
        else
        {
            displayErrorMessage(panelVoltageUnit->error());
            solarRegulatorControlDialogUi.panelVoltageDisplay->display(QString("Error"));
        }
    }
    else solarRegulatorControlDialogUi.panelVoltageDisplay->setVisible(false);
    if (batteryVoltageUnit->isConfigured())
    {
        solarRegulatorControlDialogUi.batteryVoltageDisplay->setVisible(true);
        if (ok)
        {
            solarRegulatorControlDialogUi.batteryVoltageDisplay->display(QString("%1")
                          .arg(batteryVoltage,5,'f',precision));
        }
        else
        {
            solarRegulatorControlDialogUi.batteryVoltageDisplay->display(QString("Error"));
        }
    }
    else solarRegulatorControlDialogUi.batteryVoltageDisplay->setVisible(false);
    if ((loadCurrentUnit->isConfigured()) && (panelCurrentUnit->isConfigured()))
    {
        solarRegulatorControlDialogUi.batteryCurrentDisplay->setVisible(true);
        if (ok)
        {
            solarRegulatorControlDialogUi.batteryCurrentDisplay->display(QString("%1")
                      .arg(panelCurrent-loadCurrent,5,'f',precision));
        }
        else
        {
            solarRegulatorControlDialogUi.batteryCurrentDisplay->display(QString("Error"));
        }
    }
    else solarRegulatorControlDialogUi.batteryCurrentDisplay->setVisible(false);
    if (panelCurrentUnit->isConfigured())
    {
        solarRegulatorControlDialogUi.panelCurrentDisplay->setVisible(true);
        if (ok)
        {
            solarRegulatorControlDialogUi.panelCurrentDisplay->display(QString("%1")
                          .arg(panelCurrent,5,'f',precision));
        }
        else
        {
            solarRegulatorControlDialogUi.panelCurrentDisplay->display(QString("Error"));
        }
    }
    else solarRegulatorControlDialogUi.panelCurrentDisplay->setVisible(false);
    if (ok)
    {
        solarRegulatorControlDialogUi.averageBatteryCurrentDisplay->display(QString("%1")
                      .arg(averageBatteryCurrent,5,'f',precision));
    }
    else
    {
        solarRegulatorControlDialogUi.averageBatteryCurrentDisplay->display(QString("Error"));
    }
    if (ok)
    {
        solarRegulatorControlDialogUi.averageBatteryVoltageDisplay->display(QString("%1")
                      .arg(averageBatteryVoltage,5,'f',precision));
    }
    else
    {
        solarRegulatorControlDialogUi.averageBatteryVoltageDisplay->display(QString("Error"));
    }
}

//-----------------------------------------------------------------------------
/** @brief Upload firmware Intel Hex file to the device through the bootloader.

This button sends a command to force the unit to jump to the bootloader,
aborting any actions that are currently in progress.
*/

void BatteryChargerControl::on_uploadButton_clicked()
{
    solarRegulatorControlDialogUi.errorLabel->clear();
    unit->bootloaderJump();
    uint initialBaudrate = 3;
//! The bootloader window is opened and bootloader commands are used.
    AvrSerialProg* bootloaderForm = new AvrSerialProg(unit->getPort(),initialBaudrate,FALSE,FALSE,this);
    bootloaderForm->exec();
//! After the bootloader window is closed, we must resynchronize.
    unit->synchronize(100);
    displayErrorMessage(unit->error());
}
//-----------------------------------------------------------------------------
/** @brief Configure the Microcontroller.

This button opens the configure window.
*/

void BatteryChargerControl::on_configureButton_clicked()
{
    solarRegulatorControlDialogUi.errorLabel->clear();
    BatteryChargerConfigure configureForm(unit);
    configureForm.exec();
    displayErrorMessage(unit->error());
}
//-----------------------------------------------------------------------------
/** @brief Show Microcontroller Information.

This button opens the information window.
*/

void BatteryChargerControl::on_aboutButton_clicked()
{
    solarRegulatorControlDialogUi.errorLabel->clear();
    BatteryChargerAbout aboutForm(unit);
    aboutForm.exec();
    displayErrorMessage(unit->error());
}
//-----------------------------------------------------------------------------
/** @brief  Pull down the data saved to date.

Download the DAU buffer contents to the save file. The DAU will reset the buffer
pointers, thus freeing up buffer space. If the unit had been powered off, the
pointers will be invalid. In that case use offline mode to access the pointers
in the EEPROM and so recover the data. We don't use this method normally as
reading the buffer during an active conversion process requires the pointers to
be read and immediately reset so that further writes to the buffer will not be
consistent.
*/

void BatteryChargerControl::on_downloadButton_clicked()
{
    unsigned char resultBuffer[65536];
    unsigned int startPointer = 0;
    unsigned int endPointer = 0;
    unsigned int length = 0;
    solarRegulatorControlDialogUi.errorLabel->clear();
    bool ok;
//! Check that the save file has been defined and is open.
    if (saveFile.isEmpty())
    {
        displayErrorMessage("Output File not defined");
        return;
    }
    if (! outFile->isOpen())
    {
        displayErrorMessage("Output File not open");
        return;
    }
//! If offline, we need to grab the pointers from the external EEPROM.
    if (solarRegulatorControlDialogUi.offlineCheckBox->isChecked())
    {
        ok = unit->sendDataBufferRead(resultBuffer,0x08,0x0C,
                              unit->getMaxBlockLength(),length);
        startPointer = (unsigned int)(resultBuffer[0] +
                                    (resultBuffer[1]<<8));
        endPointer   = (unsigned int)(resultBuffer[2] +
                                    (resultBuffer[3]<<8));
    }
/** Otherwise use the pointers maintained by the process. The DAU will return
the pointers and reset them to make a zero length queue. */
    else
    {
        ok = unit->sendResetBufferPointers(startPointer,endPointer);
        if (ok)
        {
            if ((startPointer < 16) && (endPointer < 16))
            {
                displayErrorMessage("Invalid Pointers - use Offline Mode");
                return;
            }
        }
    }
//! Find the number of milliseconds in a real time clock tick.
    float rtcTickMs =
            (float)(unit->getRtcFactor(unit->getRtcScale())*256)/
            (float)unit->getCpuFreq();
/** Read in the buffer data using the pointers and a simple block EEPROM read
command.*/
    if (ok)
        ok = unit->sendDataBufferRead(resultBuffer,startPointer,endPointer,
                                      unit->getMaxBlockLength(),length);
/** Then interpret the results and write the resulting buffer to the file.*/
    if (ok)
    {
/*        AcqSimpleAdcUnit* panelVoltageUnit = new AcqSimpleAdcUnit(unit,0);
        unsigned char adcPrecisionPV  = panelVoltageUnit->getAdcPrecision();
        unsigned int mulScaleFactorPV = panelVoltageUnit->getMulScaleFactor();
        unsigned int offScaleFactorPV = panelVoltageUnit->getOffScaleFactor();
        AcqSimpleAdcUnit* batteryVoltageUnit = new AcqSimpleAdcUnit(unit,1);
        unsigned char adcPrecisionBV  = batteryVoltageUnit->getAdcPrecision();
        unsigned int mulScaleFactorBV = batteryVoltageUnit->getMulScaleFactor();
        unsigned int offScaleFactorBV = batteryVoltageUnit->getOffScaleFactor();
        AcqSimpleAdcUnit* loadCurrentUnit = new AcqSimpleAdcUnit(unit,2);
        unsigned char adcPrecisionLC  = loadCurrentUnit->getAdcPrecision();
        unsigned int mulScaleFactorLC = loadCurrentUnit->getMulScaleFactor();
        unsigned int offScaleFactorLC = loadCurrentUnit->getOffScaleFactor();
        AcqSimpleAdcUnit* panelCurrentUnit = new AcqSimpleAdcUnit(unit,3);
        unsigned char adcPrecisionPC  = panelCurrentUnit->getAdcPrecision();
        unsigned int mulScaleFactorPC = panelCurrentUnit->getMulScaleFactor();
        int offScaleFactorPC = panelCurrentUnit->getOffScaleFactor();
        if (offScaleFactorPC > 64500) offScaleFactorPC -= 65536;*/
        QTextStream out(outFile);
        unsigned int n=0;
        while (n<length)
        {
            unsigned long time = (unsigned long)(resultBuffer[n] +
                                               (resultBuffer[n+1]<<8)) +
                               ((unsigned long)(resultBuffer[n+2] +
                                               (resultBuffer[n+3]<<8))<<16);
            int batteryVoltageSample = (int)(resultBuffer[n+4] +
                                               (resultBuffer[n+5]<<8));
            if (batteryVoltageSample > 64500) batteryVoltageSample -= 65536;
            int panelCurrentSample = (int)(resultBuffer[n+6] +
                                               (resultBuffer[n+7]<<8));
            if (panelCurrentSample > 64500) panelCurrentSample -= 65536;
            out << startTime.addMSecs(time*rtcTickMs).time().toString("hh:mm:ss") << ",";
            out << (batteryVoltageSample)/655.36 << ",";
            out << (panelCurrentSample)/655.36;
            out << endl;
            n += 16;
        }
    }
    if (! ok) displayErrorMessage(unit->error());
}
//-----------------------------------------------------------------------------
/** @brief Obtain a save file name and path and attempt to open it.
*/

void BatteryChargerControl::on_saveFileButton_clicked()
{
//! Make sure there is no file already open.
    if (! saveFile.isEmpty())
    {
        displayErrorMessage("A file is already open - close it first");
        return;
    }
    solarRegulatorControlDialogUi.errorLabel->clear();
    QString filename = QFileDialog::getSaveFileName(this,
                        "Save Acquired Data",
                        QString(),
                        "Comma Separated Variables (*.csv)",0,
                        QFileDialog::DontConfirmOverwrite);
    if (filename.isEmpty()) return;
    if (! filename.endsWith(".csv")) filename.append(".csv");
    QFileInfo fileInfo(filename);
    saveDirectory = fileInfo.absolutePath();
    saveFile = saveDirectory.filePath(filename);
    outFile = new QFile(saveFile);             // Open file for output
/** If the output file exists, then we are asking for it to be updated with new
data from the Unit. We also need to check that it is a valid data file.*/
    if (outFile->exists())
    {
        if (! outFile->open(QIODevice::ReadOnly))
        {
            displayErrorMessage("Could not open output file to read");
            return;
        }
        outFile->close();
        if (! outFile->open(QIODevice::Append))
        {
            displayErrorMessage("Could not open output file to append");
            return;
        }
    }
/** If the output file does not exist, then create it. */
    else
    {
        if (! outFile->open(QIODevice::WriteOnly))
        {
            displayErrorMessage("Could not open the output file");
            return;
        }
        qDebug() << "Save File Created";
    }
    solarRegulatorControlDialogUi.downloadButton->setEnabled(true);
    solarRegulatorControlDialogUi.closeFileButton->setEnabled(true);
}
//-----------------------------------------------------------------------------
/** @brief Close the save file.
*/
void BatteryChargerControl::on_closeFileButton_clicked()
{
    solarRegulatorControlDialogUi.errorLabel->clear();
    if (saveFile.isEmpty())
        displayErrorMessage("File already closed");
    else
    {
        outFile->close();
        delete outFile;
//! Wipe the name to prevent same file being used.
        saveFile = QString();
        qDebug() << "Save File Closed";
//! Disable the buttons
        solarRegulatorControlDialogUi.downloadButton->setEnabled(false);
        solarRegulatorControlDialogUi.closeFileButton->setEnabled(false);
    }
}
//-----------------------------------------------------------------------------
/** @brief Show an error condition in the Error label.

@param[in] message Message to be displayed.
*/

void BatteryChargerControl::displayErrorMessage(const QString message)
{
    solarRegulatorControlDialogUi.errorLabel->setText(message);
}
//-----------------------------------------------------------------------------
/** @brief Successful Synchronization.

@return Boolean indication that synchronization to the unit was successful.
*/
bool BatteryChargerControl::success()
{
    return synchronized;
}
//-----------------------------------------------------------------------------
/** @brief Error Message.

This provides access to error messages from outside the class.
@return The latest error message that was detected.
*/
QString BatteryChargerControl::error()
{
    return errorMessage;
}
/**@}*/
//-----------------------------------------------------------------------------
