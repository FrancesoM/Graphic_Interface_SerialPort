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

#ifndef CONSOLE_H
#define CONSOLE_H

#include <QPlainTextEdit>
#include "qcustomplot.h"

#include <math.h>

#define NCHANNEL 8
#define HISTORY 50

//Define here the channels used for recognition, if N_channel changes, remember to
//add or remove the corresponding channel
#define N_CHANNEL_FOR_RECOGNITION 3
#define CHANNEL_RECOGNITION_1 3
#define CHANNEL_RECOGNITION_2 5
#define CHANNEL_RECOGNITION_3 7

//We'd like a matrix, where the element i,j is the distance between the channel
//i to j, but actually we need only the upper part since when i=j distance is always 0
//and the element i,j is equal to j,i, so we count the number of elements upper diagonal
//and we create a vector to take all these elements.
#define N_DISTANCES (N_CHANNEL_FOR_RECOGNITION*(N_CHANNEL_FOR_RECOGNITION-1))/2



class Console : public QCustomPlot
{
    Q_OBJECT

public:
    explicit Console(int n_ch, int y_min, int y_max, QCustomPlot *parent = nullptr);

    void putData(QVector<float> data);

     void clearData();

    void setLocalEchoEnabled(bool set);



protected:
    void keyPressEvent(QKeyEvent *e) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void mouseDoubleClickEvent(QMouseEvent *e) Q_DECL_OVERRIDE;
    void contextMenuEvent(QContextMenuEvent *e) Q_DECL_OVERRIDE;

private:
    bool localEchoEnabled;
    int n_channels;
    //We need the vecto
    //QVector < double > distance = QVector<double>(N_DISTANCES);
    //QVector < Qvector<int> > indexes = QVector < QVector<int>(2) >(N_DISTANCES);
    QVector< QQueue<double> > channels = QVector< QQueue<double> >(HISTORY);
    QVector<QColor> colorVector = {Qt::black,   //0
                                   Qt::blue,    //1
                                   Qt::red,     //2
                                   Qt::green,   //3 <---
                                   Qt::magenta, //4
                                   Qt::yellow,  //5 <---
                                   Qt::gray,    //6
                                   Qt::cyan};   //7 <---


};

#endif // CONSOLE_H
