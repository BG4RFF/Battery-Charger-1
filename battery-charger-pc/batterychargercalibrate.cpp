/*
Title:    Atmel Microcontroller Battery Charger Project
*/
/** @defgroup batterychargercalibration Battery Charger Calibration Window

These functions provide functionality for the Battery Charger Calibration
Window in which the analogue channels are calibrated against known voltages
and currents, and conversion parameters are computed and programmed into the
channel information blocks of the unit.

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

#include "batterychargercalibrate.h"
#include <QDebug>

//-----------------------------------------------------------------------------
/** @brief Constructor.

Check if the channel exists in the unit. If not, create a default entry.
We need to do this because the unit will not present any measurements until it
has a channel to work on.

@param u Acquisition Unit Object
@param parent Parent widget.
*/
BatteryChargerCalibrate::BatteryChargerCalibrate(AcqSimpleAdcUnit* u,
          QWidget* parent) : QDialog(parent)
{
    channelUnit = u;
// Create a dummy entry if necessary
    if (! channelUnit->isConfigured())
    {
        bool ok = channelUnit->programInformationBlock(DEFAULT_AD_PRECISION,0,false,0,0);
        if (! ok) displayErrorMessage("Could not program device with default channel");
    }
//! Build the User Interface display from Ui object.
    BatteryChargerCalibrateDialogueUi.setupUi(this);
    if (channelUnit->getChannel() == 0)
        BatteryChargerCalibrateDialogueUi.subTitle->setText("Panel Voltage");
    else if (channelUnit->getChannel() == 1)
        BatteryChargerCalibrateDialogueUi.subTitle->setText("Battery Voltage");
    else if (channelUnit->getChannel() == 2)
        BatteryChargerCalibrateDialogueUi.subTitle->setText("Panel Current");
    else if (channelUnit->getChannel() == 3)
        BatteryChargerCalibrateDialogueUi.subTitle->setText("Load Current");
    firstEntered = false;
// Blank out the buttons until some entries are made
    on_clearEntriesButton_clicked();
}

BatteryChargerCalibrate::~BatteryChargerCalibrate()
{
}

//-----------------------------------------------------------------------------
/** @brief Close when OK is activated returning success.
*/

void BatteryChargerCalibrate::on_closeButton_clicked()
{
    accept();
}

//-----------------------------------------------------------------------------
/** @brief Enable the first entry button when the text changes.

Only if the entered value makes a valid floating point number. If it is not valid
then disable it.
*/

void BatteryChargerCalibrate::on_firstValueLineEdit_textChanged()
{
    bool firstValueOK;
    firstValue = BatteryChargerCalibrateDialogueUi.firstValueLineEdit->text().toFloat(&firstValueOK);
    if (firstValueOK)
        BatteryChargerCalibrateDialogueUi.firstValueSetButton->setEnabled(true);
    else
        BatteryChargerCalibrateDialogueUi.firstValueSetButton->setEnabled(false);
}

//-----------------------------------------------------------------------------
/** @brief Enable the compute button when the text changes.

Only if the entered value makes a valid floating point number, and the first value is
also valid and has been successfully entered. If it is not valid, then disable it.
*/

void BatteryChargerCalibrate::on_secondValueLineEdit_textChanged()
{
    bool firstValueOK;
    bool secondValueOK;
    secondValue = BatteryChargerCalibrateDialogueUi.secondValueLineEdit->text().toFloat(&secondValueOK);
    firstValue = BatteryChargerCalibrateDialogueUi.firstValueLineEdit->text().toFloat(&firstValueOK);
    if (secondValueOK && firstValueOK && firstEntered)
        BatteryChargerCalibrateDialogueUi.secondValueSetButton->setEnabled(true);
    else
        BatteryChargerCalibrateDialogueUi.secondValueSetButton->setEnabled(false);
}

//-----------------------------------------------------------------------------
/** @brief Read the first calibration value from the unit.

The unit is interrogated for the smoothed A/D reading for the channel to match
with the first measurement. The second value set button is then enabled and the
first is disabled.
*/

void BatteryChargerCalibrate::on_firstValueSetButton_clicked()
{
    unsigned char s;
    unsigned char stateVariables[100];
    unsigned char item = (channelUnit->getChannel() << 1);
    bool ok = channelUnit->getUnit()->getStateVariables(0,stateVariables,s);
    firstSample = (stateVariables[item+1] << 8) + stateVariables[item];
    if (ok)
    {
        BatteryChargerCalibrateDialogueUi.secondValueSetButton->setEnabled(true);
        BatteryChargerCalibrateDialogueUi.firstValueSetButton->setEnabled(false);
        firstEntered = true;
    }
    if (! ok) displayErrorMessage("Could not read state measurement");
}

//-----------------------------------------------------------------------------
/** @brief Read the second calibration value from the unit.

The unit is interrogated for the smoothed A/D reading for the channel. This is
then combined with the first value to determine the scaling factors and the maximum
and minimum of the range is displayed. The commit button is enabled.
*/

void BatteryChargerCalibrate::on_secondValueSetButton_clicked()
{
    unsigned char s;
    unsigned char stateVariables[100];
    unsigned char item = (channelUnit->getChannel() << 1);
    bool ok = channelUnit->getUnit()->getStateVariables(0,stateVariables,s);
    secondSample = (stateVariables[item+1] << 8) + stateVariables[item];
    if (ok)
    {
        BatteryChargerCalibrateDialogueUi.firstValueSetButton->setEnabled(false);
        BatteryChargerCalibrateDialogueUi.secondValueSetButton->setEnabled(false);
        BatteryChargerCalibrateDialogueUi.commitButton->setEnabled(true);
        float VL = (secondValue*firstSample - firstValue*secondSample)/((float)firstSample - (float)secondSample);
        float VHmVL = (firstValue - secondValue)/((float)firstSample - (float)secondSample)*(1 << channelUnit->getAdcPrecision());
        qDebug() << "Low" << VL << "High-Low" << VHmVL;
        BatteryChargerCalibrateDialogueUi.maximumLcdNumber->display(VHmVL+VL);
        BatteryChargerCalibrateDialogueUi.minimumLcdNumber->display(VL);
        channelUnit->setMulScaleFactor((unsigned int)(VHmVL*655.36));
        channelUnit->setOffScaleFactor((unsigned int)(VL*655.36));
        BatteryChargerCalibrateDialogueUi.commitButton->setEnabled(true);    
    }
    if (! ok) displayErrorMessage("Could not read state measurement");
}

//-----------------------------------------------------------------------------
/** @brief Clear all the entries and start again.

*/

void BatteryChargerCalibrate::on_clearEntriesButton_clicked()
{
    BatteryChargerCalibrateDialogueUi.firstValueSetButton->setEnabled(false);
    BatteryChargerCalibrateDialogueUi.secondValueSetButton->setEnabled(false);
    BatteryChargerCalibrateDialogueUi.commitButton->setEnabled(false);   
    BatteryChargerCalibrateDialogueUi.firstValueLineEdit->setText(""); 
    BatteryChargerCalibrateDialogueUi.secondValueLineEdit->setText(""); 
}

//-----------------------------------------------------------------------------
/** @brief Commit the scaling factors to the unit.
*/

void BatteryChargerCalibrate::on_commitButton_clicked()
{
    bool ok = channelUnit->programInformationBlock(channelUnit->getAdcPrecision(),
                                         0,false,
                                         channelUnit->getMulScaleFactor(),
                                         channelUnit->getOffScaleFactor());
    if (! ok) displayErrorMessage("Could not program device");
}
//-----------------------------------------------------------------------------
/** @brief Error Message.

This provides access to error messages from outside the class.
@return The latest error message that was detected.
*/
void BatteryChargerCalibrate::displayErrorMessage(const QString message)
{
     BatteryChargerCalibrateDialogueUi.errorLabel->setText(message);
}
/**@}*/
//-----------------------------------------------------------------------------
