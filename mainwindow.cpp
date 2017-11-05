/****************************************************************************
**
** Copyright (C) 2012 Denis Shienkov <denis.shienkov@gmail.com>
** Copyright (C) 2012 Laszlo Papp <lpapp@kde.org>
** Contact: https://www.qt.io/licensing/
**
** This file is part of the QtSerialPort module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "console.h"
#include "settingsdialog.h"
#include <classifier.h>

#include <QMessageBox>
#include <QLabel>
#include <QtSerialPort/QSerialPort>
#include <QDebug>

//! [0]
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
//! [0]
    ui->setupUi(this);
    console = new Console(NUM_AXES,DOWN_AXIS,UP_AXIS);
    console->setEnabled(false);

    consoleGyro = new Console(1,-180,180);
    consoleGyro->setEnabled(false);

    ui->TwoGraphsLayout->addWidget(console);
    ui->TwoGraphsLayout->addWidget(consoleGyro);


//! [1]
    serial = new QSerialPort(this);
    serialGyro = new QSerialPort(this);
//! [1]
    settings = new SettingsDialog("COM10","115200");
    settingsGyro = new SettingsDialog("COM4","115200");
    flagGyro = 0;

    classifier = new Classifier;

    ui->actionConnect->setEnabled(true);
    ui->actionConnectGyro->setEnabled(true);
    ui->actionDisconnect->setEnabled(false);
    ui->actionDisconnectGyro->setEnabled(false);

    ui->actionQuit->setEnabled(true);

    ui->actionConfigure->setEnabled(true);
    ui->actionConfigureGyro->setEnabled(true);

    status = new QLabel;
    ui->statusBar->addWidget(status);

    ui->StartButton->setIcon(QIcon(":/images/play.png"));
    ui->StopButton->setIcon(QIcon(":/images/end.png"));
    ui->CalibrateButton->setIcon(QIcon(":/images/cal.png"));

    initActionsConnections();

    this->flag_can_save = 0;
    this->counter_patient = 0;

    connect(serial, static_cast<void (QSerialPort::*)(QSerialPort::SerialPortError)>(&QSerialPort::error),
            this, &MainWindow::handleError);

//! [2]
    connect(serial, &QSerialPort::readyRead, this, &MainWindow::readData);
    connect(serialGyro, &QSerialPort::readyRead, this, &MainWindow::readDataGyro);
//! [3]
}
//! [3]

MainWindow::~MainWindow()
{
    delete settings;
    delete ui;
    classifier->hide();
    delete classifier;

}

//! [4]
void MainWindow::openSerialPort(SettingsDialog *s,
                                QSerialPort *port,
                                Console* c,
                                QString msg)
{
    SettingsDialog::Settings p = s->settings();
    port->setPortName(p.name);
    port->setBaudRate(p.baudRate);
    port->setDataBits(p.dataBits);
    port->setParity(p.parity);
    port->setStopBits(p.stopBits);
    port->setFlowControl(p.flowControl);
    if (port->open(QIODevice::ReadWrite)) {
        c->setEnabled(true);
        c->setLocalEchoEnabled(p.localEchoEnabled);
        if(msg == "UH" ){
            ui->actionConnect->setEnabled(false);
            ui->actionDisconnect->setEnabled(true);
            ui->actionConfigure->setEnabled(false);
            showStatusMessage(tr("Connected to UH %1 : %2, %3, %4, %5, %6")
                              .arg(p.name).arg(p.stringBaudRate).arg(p.stringDataBits)
                              .arg(p.stringParity).arg(p.stringStopBits).arg(p.stringFlowControl));
        } else{
            ui->actionConnectGyro->setEnabled(false);
            ui->actionDisconnectGyro->setEnabled(true);
            ui->actionConfigureGyro->setEnabled(false);
            showStatusMessage(tr("Connected to Gyro %1 : %2, %3, %4, %5, %6")
                              .arg(p.name).arg(p.stringBaudRate).arg(p.stringDataBits)
                              .arg(p.stringParity).arg(p.stringStopBits).arg(p.stringFlowControl));
        }

    } else {
        QMessageBox::critical(this, tr("Error"), port->errorString());

        showStatusMessage(tr("Open error"));
    }
}
//! [4]

//! [5]
void MainWindow::closeSerialPort(QSerialPort* port, Console *c, QString msg)
{
    if (port->isOpen())
        port->close();
    c->setEnabled(false);
    if(msg == "UH"){
        ui->actionConnect->setEnabled(true);
        ui->actionDisconnect->setEnabled(false);
        ui->actionConfigure->setEnabled(true);
    }
    else{
        ui->actionConnectGyro->setEnabled(true);
        ui->actionDisconnectGyro->setEnabled(false);
        ui->actionConfigureGyro->setEnabled(true);
    }
    showStatusMessage(tr("Disconnected"));
}
//! [5]
//!

void MainWindow::closeSerialPort()
{
    closeSerialPort(serial,console,"UH");
    closeSerialPort(serialGyro,consoleGyro,"Gyro");
}

void MainWindow::about()
{
    QMessageBox::about(this, tr("About Simple Terminal"),
                       tr("The <b>Simple Terminal</b> example demonstrates how to "
                          "use the Qt Serial Port module in modern GUI applications "
                          "using Qt, with a menu bar, toolbars, and a status bar."));
}

//! [6]

//! [7]
//void MainWindow::readData()
//{
//    qint16 temp;
//    QByteArray data = serial->readAll();
//    //Process the array, in this case simply show it on the screen.
//    for(int i = 0;i<data.length();i++){
//        //qDebug() << i << ": " << QString::number(data[i],16) << "\n";
//        this->bytesQueue.enqueue(data[i]);
//    }

//    if(this->bytesQueue.size() == 16){
//        //qDebug() << this->bytesQueue << "\n";
//        for(int i = 0;i<NCHANNEL;i++){
//            quint8 temp_lsb = this->bytesQueue.dequeue();
//            quint8 temp_msb = this->bytesQueue.dequeue();
//            temp = (temp_msb << 8) | (temp_lsb);
//            this->yVec[i] = (float)temp;

//        }
//        qDebug() << this->yVec << "\n";
//        console->putData(this->yVec);
//    }

//    //If flag is on, write the data onto a file
//    if(this->flag_can_save){
//        if(this->recording != nullptr){
//            QTextStream out(this->recording);

//            for(int i = 0;i<yVec.length();i++)
//                out << yVec[i];

//            out<<"\n";
//        }
//    }

//}

void MainWindow::readData()
{
    /*********************************************************************************
     *                    TEMPLATE TO READ THE DESIRED PACKETS                       *
     *                  CHANGE ONLY PARTS THAT START WITH "User:"                    *
     *********************************************************************************/

    /*********************************************************************************
     *    User:Choose your packet size and create container arrays to store them     *
     *********************************************************************************/

    //Packet size = 2(start) + 8x2(channels) + 3x2(angles) + 3 = 2+16+6+3 = 27

//    char receivedPhotoReflectors[16]; //8 values, qint16 each
//    char receivedXangle[2]; //qint16 (int is 2 bytes long on avr micros)
//    char receivedYangle[2];
//    char receivedZangle[2];

//    qint16 reconstructedXangle;
//    qint16 reconstructedYangle;
//    qint16 reconstructedZangle;

//    int packetSize = 27;
//    int nData = NCHANNEL;
//    QVector<float> yVec = QVector<float>(nData);

    //Packet size = 2(start) + 3*2(x,y,z) + 3(stop) = 11

    quint8 receivedAxis[6];
    int packetSize = 11;
    int nData = 3;  //x, y, z
    QVector<float> yVec = QVector<float>(nData);
    QVector<uint16_t> temp = QVector<uint16_t>(nData);

    /*********************************************************************************
     *                      Start to collect data inside the queue                   *
     *********************************************************************************/

    QByteArray data = serial->readAll();
    for(int i = 0;i<data.length();i++){
        //qDebug() << i << ": " << QString::number(data[i],16) << "\n";
        this->bytesQueue.enqueue(data[i]);
    }

    /*********************************************************************************
     *                      Start to decode data in the queue                        *
     *********************************************************************************/

    if(bytesQueue.length() >= packetSize)
    {
        for(int offset = 0;offset < bytesQueue.length() - packetSize;offset++)
        {
            if(bytesQueue[offset] == 0x00 &&
               bytesQueue[offset+1] == 0x00 &&
               bytesQueue[offset+packetSize-3] == 0x21 &&
               bytesQueue[offset+packetSize-2] == 0x21 &&
               bytesQueue[offset+packetSize-1] == 0x21 )
            {

    /*********************************************************************************
     *         User:Manually transfer bytes from queue to the container array        *
     *********************************************************************************/

//                //MSB comes first but in architecture it is stored last
//                receivedPhotoReflectors[0] = bytesQueue[offset+2];      //--
//                receivedPhotoReflectors[1] = bytesQueue[offset+3];      //ch1
//                receivedPhotoReflectors[2] = bytesQueue[offset+4];      //--
//                receivedPhotoReflectors[3] = bytesQueue[offset+5];      //ch2
//                receivedPhotoReflectors[4] = bytesQueue[offset+6];      //--
//                receivedPhotoReflectors[5] = bytesQueue[offset+7];      //ch3
//                receivedPhotoReflectors[6] = bytesQueue[offset+8];      //--
//                receivedPhotoReflectors[7] = bytesQueue[offset+9];      //ch4
//                receivedPhotoReflectors[8] = bytesQueue[offset+10];     //--
//                receivedPhotoReflectors[9] = bytesQueue[offset+11];     //ch5
//                receivedPhotoReflectors[10] = bytesQueue[offset+12];     //--
//                receivedPhotoReflectors[11] = bytesQueue[offset+13];     //ch6
//                receivedPhotoReflectors[12] = bytesQueue[offset+14];     //--
//                receivedPhotoReflectors[13] = bytesQueue[offset+15];     //ch7
//                receivedPhotoReflectors[14] = bytesQueue[offset+16];     //--
//                receivedPhotoReflectors[15] = bytesQueue[offset+17];     //ch8

//                receivedXangle[0] = bytesQueue[offset+18];
//                receivedXangle[1] = bytesQueue[offset+19];
//                receivedYangle[0] = bytesQueue[offset+20];
//                receivedYangle[1] = bytesQueue[offset+21];
//                receivedZangle[0] = bytesQueue[offset+22];
//                receivedZangle[1] = bytesQueue[offset+23];

                  receivedAxis[0] = bytesQueue[offset+2]+0x0A;
                  receivedAxis[1] = bytesQueue[offset+3];
                  receivedAxis[2] = bytesQueue[offset+4];
                  receivedAxis[3] = bytesQueue[offset+5];
                  receivedAxis[4] = bytesQueue[offset+6];
                  receivedAxis[5] = bytesQueue[offset+7];

                  for(int i = 0;i<nData*2;i++){
                      qDebug("%04x",receivedAxis[i]); // << receivedAxis[i];
                  }

                  qDebug() << "---------------------------";


                //Now save the values
                for(int i = 0;i<nData;i++)
                {
                    //The reinterpret cast is needed to convert the bytes into the real number
                    //Then we cast to float to pass the correct type to putData
                    //yVec[i] = (float)*reinterpret_cast<quint16*>(&receivedAxis[2*i]);
                    temp[i] = (((uint16_t) receivedAxis[2*i])<<8) + receivedAxis[2*i+1];
                    yVec[i] = (float) temp[i];
                    //qDebug() << yVec[i];

                }
                console->putData(yVec);
                qDebug() << yVec;

//                reconstructedXangle = *reinterpret_cast<qint16*>(receivedXangle);
//                reconstructedYangle = *reinterpret_cast<qint16*>(receivedYangle);
//                reconstructedZangle = *reinterpret_cast<qint16*>(receivedZangle);

//                classifier->transform(reconstructedXangle,
//                                      reconstructedYangle,
//                                      reconstructedZangle,
//                                      "absolute");

                //Delete from the queue everything that has been used so far
                for(int del = 0; del < offset + packetSize; del ++){
                    bytesQueue.dequeue();
                }

            }
        }
    }



}


void MainWindow::readDataGyro()
{
    char received[4];
    QVector3D axisRotation(0.0f,0.0f,1.0f);

    QByteArray data = serialGyro->readAll();
    for(int i = 0;i<data.length();i++){
        //qDebug() << i << ": " << QString::number(data[i],16) << "\n";
        this->bytesQueueGyro.enqueue(data[i]);
    }

    //stesso motivo di sotto, se è più piccolo del frame nn parte neanche il for
    if(bytesQueueGyro.length() >= (3+2+4) ){
        //tolgo la lunghezza del frame perchè nn ha senso e andrei outofbonds
        for(int offset = 0;offset <= bytesQueueGyro.length() - (3+2+4); offset++){
            if( bytesQueueGyro[offset]==0x00 &&
                bytesQueueGyro[offset+1] == 0x00 &&
                bytesQueueGyro[offset+6] == 0x21 &&
                bytesQueueGyro[offset+7] == 0x21 &&
                bytesQueueGyro[offset+8] == 0x21 ){

                    received[3] = bytesQueueGyro[offset+5];
                    received[2] = bytesQueueGyro[offset+4];
                    received[1] = bytesQueueGyro[offset+3];
                    received[0] = bytesQueueGyro[offset+2];

                    //Save the value
                    currentAngle[0] = *(reinterpret_cast<float*>(received));
                    //Plot the angle
                    qDebug() << currentAngle;
                    consoleGyro->putData(currentAngle);

                    //Clean the queue up to the offset + the frame
                    for(int i = 0;i<offset+(3+2+4);i++) bytesQueueGyro.dequeue();

            }
        }
    }

//    if(bytesQueueGyro.length() > 9){
//        //Read and in also go one step further (eliminate the first element)
//        if(bytesQueueGyro.dequeue() == 0x00){
//            if( (bytesQueueGyro[0] == 0x00) && (bytesQueueGyro[5] == 0x21) && (bytesQueueGyro[6] == 0x21) && (bytesQueueGyro[7] == 0x21) ){
//                //currentAngle[0] = (bytesQueueGyro[4] << 24) | (bytesQueueGyro[3] << 16) | (bytesQueueGyro[2] << 8) | (bytesQueueGyro[1]);
//                received[3] = bytesQueueGyro[4];
//                received[2] = bytesQueueGyro[3];
//                received[1] = bytesQueueGyro[2];
//                received[0] = bytesQueueGyro[1];

//                currentAngle[0] = *(reinterpret_cast<float*>(received));

//                consoleGyro->putData(currentAngle);
//            }
//        }

//    }


}

void MainWindow::openUH()
{
    openSerialPort(settings,serial,console,"UH");
}

void MainWindow::closeUH()
{
    closeSerialPort(serial,console,"UH");
}

void MainWindow::openGyro()
{
    openSerialPort(settingsGyro,serialGyro,consoleGyro,"Gyro");
}

void MainWindow::closeGyro()
{
    closeSerialPort(serialGyro,consoleGyro,"Gyro");
}
//! [7]

//! [8]!
void MainWindow::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ResourceError) {
        QMessageBox::critical(this, tr("Critical Error"), serial->errorString());
        closeSerialPort();
    }
}

void MainWindow::on_StartButton_clicked()
{
    if(this->flag_can_save){

        this->filename = ui->PatientName->text();
        this->filename.append(QString::number(this->counter_patient));
        this->filename.append(".txt");
        this->filename.prepend("C:/Users/Francesco/Documents/GUI-Serial/");

        qDebug() << this->filename;

        this->recording = new QFile(this->filename);
        if (!recording->open(QIODevice::WriteOnly | QIODevice::Text))
        {qDebug() << "Unable to open the file" ;
        }    //Handle this

        serial->write(QString("s").toLocal8Bit());
    }

}

void MainWindow::on_StopButton_clicked()
{
    serial->write(QString("e").toLocal8Bit());

    if(this->flag_can_save){

        this->counter_patient++;

        //Garbage collector
        //Prevent to delete again if change button was pushed before
        //The delete function remove the allocated memory, but the pointer
        //keeps his address, so we need to force it to null.
        if(this->recording != nullptr){
            this->recording->close();
            delete this->recording;
            this->recording = NULL;

        }
    }


}

void MainWindow::on_CalibrateButton_clicked()
{
    serial->write(QString("c").toLocal8Bit());

    //If there isn't the name of the patient, don't trigger the flag
    //If the calibration is done, then open a file with the name of the patient
    //plus the counter
    if( !(ui->PatientName->text().isEmpty()) ){
        this->flag_can_save=1;
        ui->CalibrateButton->setStyleSheet("background-color: rgb(0, 222, 37)");
    }



}

void MainWindow::on_ChangeButton_clicked()
{
    serial->write(QString("e").toLocal8Bit());
    this->flag_can_save = 0;
    this->counter_patient = 0;

    this->console->clearData();
    ui->PatientName->clear();
    ui->CalibrateButton->setStyleSheet("background-color: rgb(255, 47, 6)");

    //Garbage collector
    //Prevent to delete again if stop button was pushed before
    //The delete function remove the allocated memory, but the pointer
    //keeps his address, so we need to force it to null.
    if(this->recording != nullptr){
        qDebug() << "I'm about to delete an already deleted pointer";
        this->recording->close();
        delete this->recording;
        this->recording = NULL;

    }

}

//! [8]

void MainWindow::initActionsConnections()
{
    connect(ui->actionConnect, &QAction::triggered, this, &MainWindow::openUH);
    connect(ui->actionConnectGyro, &QAction::triggered, this, &MainWindow::openGyro);

    connect(ui->actionDisconnect, &QAction::triggered, this, &MainWindow::closeUH);
    connect(ui->actionDisconnectGyro, &QAction::triggered, this, &MainWindow::closeGyro);

    connect(ui->actionQuit, &QAction::triggered, this, &MainWindow::close);

    connect(ui->actionConfigure, &QAction::triggered, settings, &SettingsDialog::show);
    connect(ui->actionConfigureGyro, &QAction::triggered, settingsGyro, &SettingsDialog::show);

    //connect(ui->actionClear, &QAction::triggered, console, &Console::clear);
    connect(ui->actionAbout, &QAction::triggered, this, &MainWindow::about);
    connect(ui->actionAboutQt, &QAction::triggered, qApp, &QApplication::aboutQt);

    connect(ui->actionTranslate, &QAction::triggered, this, &MainWindow::broadcastTransformation);
}

void MainWindow::broadcastTransformation()
{

    classifier->transform(ui->LineX->text().toFloat(),
                          ui->LineY->text().toFloat(),
                          ui->LineZ->text().toFloat(),
                          "successive");

}

void MainWindow::showStatusMessage(const QString &message)
{
    status->setText(message);
}
