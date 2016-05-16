#include "controlvw.h"

controlVW::controlVW(QWidget *parent) :
    QWidget(parent)
{

    QGridLayout* mainLayout = new QGridLayout();

    setLayout(mainLayout);

    setAutoFillBackground(true);

//    show();

    QPalette Pal(palette());
    // set black background
    Pal.setColor(QPalette::Background, Qt::black);
    setAutoFillBackground(true);
    setPalette(Pal);

    setMinimumSize(400, 400);
    qimage.load(":/images/pelota.png");

}

void controlVW::Stop()
{
    line = QPointF(0, 0);
    repaint();

}


void controlVW::mouseMoveEvent ( QMouseEvent * event )
{

    if(event->buttons()==Qt::LeftButton ){

        int x = event->x()-width()/2;
        int y = event->y()-height()/2;

        line = QPointF(x, y);

        repaint();

    }
}

void controlVW::paintEvent(QPaintEvent *)
{

   int _width = width();
   int _height = height();


   int width = 2;
   QPen pen;

   QPainter painter(this);
   painter.setPen(pen);

   pen = QPen(Qt::blue, width);
   painter.setPen(pen);

   // Centro del widget
   painter.translate(QPoint(_width/2, _height/2));

   // eje
   painter.drawLine(QPointF(-_width, 0),
                    QPointF( _width, 0));

   painter.drawLine(QPointF(0, -_height),
                    QPointF(0, _height));

   // con el raton
   pen = QPen(Qt::red, width);
   painter.setPen(pen);

   painter.drawLine(QPointF(line.x(), -_height),
                    QPointF(line.x(), _height));

   painter.drawLine(QPointF(-_width, line.y()),
                    QPointF( _width, line.y()));

//   std::cout << "x: " << line.x() << " y: " << -line.y() << std::endl;

   float k = 0.01;
   float p = 0;

   float v_normalized = 40 * (k * line.y() + p)*(-1);
   float w_normalized = 20 * (k * line.x() + p)*(-1);

//   std::cout << "v_normalized: " << v_normalized << " w_normalized: " << w_normalized << std::endl;

   emit VW_changed((int)v_normalized, (int)w_normalized);

   painter.drawImage(line.x()-qimage.width()/2, line.y()-qimage.height()/2, qimage);

}
