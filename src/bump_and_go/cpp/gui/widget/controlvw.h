#ifndef CONTROLVW_H
#define CONTROLVW_H

#include <QtGui>

#include <iostream>

class controlVW : public QWidget
{
    Q_OBJECT
public:
    explicit controlVW(QWidget *parent = 0);
    
    void Stop();

private:
    QPointF line;
    QImage qimage;

protected:
    void paintEvent(QPaintEvent *);
    void mouseMoveEvent ( QMouseEvent * event );

signals:
    void VW_changed(int, int);

public slots:
    
};

#endif // CONTROLVW_H
