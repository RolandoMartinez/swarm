/*!
<<<<<<< HEAD
 * \brief  This frame is intended to show information about the quality of
 *         the GPS sensor data, for example, the number of satellites current
 *         sending data.
 * \author Matthew Fricke
 * \date   November 11th 2015
 * \todo   This frame is not currently being used and is currently just a place
 *         holder class.
 * \class  GPSFrame
=======
 * \brief   This frame is intended to show information about the quality of the GPS sensor data,
 *          for example, the number of satellites current sending data.
 * \author  Matthew Fricke
 * \date    November 11th 2015
 * \todo    This frame is not currently being used and is currently just a place holder class.
 * \class   GPSFrame
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
 */

#ifndef GPSFRAME_H
#define GPSFRAME_H

#include <QTime> // for frame rate
#include <QFrame>
#include <QImage>
#include <QMutex>
#include <QPainter>
#include <vector>
<<<<<<< HEAD
#include <utility> // for STL pair
=======
#include <utility> // For STL pair
>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea

using namespace std;

namespace rqt_rover_gui
{
<<<<<<< HEAD
  class GPSFrame : public QFrame
  {
    Q_OBJECT

    public:
      GPSFrame(QWidget *parent, Qt::WFlags = 0);

    signals:
      void delayedUpdate();
      
    public slots:
      // currently, no class-defined slots are being used

    protected:
      void paintEvent(QPaintEvent *event);

    private:
      QTime frame_rate_timer;
      int frames;
  };
=======

class GPSFrame : public QFrame
{
    Q_OBJECT
public:
    GPSFrame(QWidget *parent, Qt::WFlags = 0);


signals:

    void delayedUpdate();

public slots:


protected:

    void paintEvent(QPaintEvent *event);

private:

    QTime frame_rate_timer;
    int frames;

};

>>>>>>> 5e1b6536af46e99b611ef960ac01a8f0043e35ea
}

#endif // GPSFrame_H
