#include "glwidget.h"

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

GLWidget::GLWidget(StateGUI *stategui, Robot *robot, QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{
    this->stategui = stategui;
    this->robot = robot;

    this->glcam_pos_X = -100.;
    this->glcam_pos_Y = -100.;
    this->glcam_pos_Z = 70.;
    this->glcam_foa_X = 0.;
    this->glcam_foa_Y = 0.;
    this->glcam_foa_Z = 0.;

    this->lati = 0.1;
    this->longi = -1.0;

    this->old_x = 0.0;
    this->old_y = 0.0;

    this->radius = 20.0;

    this->glcam_pos_X = 40.2625;
    this->glcam_pos_Y = 2.19594;
    this->glcam_pos_Z = 54.2651;
    this->glcam_foa_X = 26.3395;
    this->glcam_foa_Y = 1.63874;
    this->glcam_foa_Z = 39.918;
    this->lati = 0.91738e-07;
    this->longi = 0.619999;

    width = 600;
    height = 400;

    setWindowTitle("3D world");

}

GLWidget::~GLWidget()
{
}

QSize GLWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize GLWidget::sizeHint() const
{
    return QSize(width, height);
}

void GLWidget::initializeGL()
{
    GLfloat ambient[] = {1.0, 1.0, 1.0, 1.0};
    GLfloat diffuse[] = {1.0, 1.0, 1.0, 1.0};
    GLfloat position[] = {0.0, 3.0, 3.0, 0.0};
    GLfloat lmodel_ambient[] = {0.2, 0.2, 0.2, 1.0};
    GLfloat local_view[] = {0.0};

    glViewport(0, 0, (GLint) width, (GLint) height);
    glDrawBuffer(GL_BACK);
    glClearColor(0.6f, 0.8f, 1.0f, 0.0f);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* With this, the pioneer appears correctly, but the cubes don't */
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, position);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
    glLightModelfv(GL_LIGHT_MODEL_LOCAL_VIEWER, local_view);
    glEnable(GL_LIGHT0);
    // glEnable (GL_LIGHTING);
    glEnable(GL_POINT_SMOOTH);

    glEnable(GL_TEXTURE_2D); // Enable Texture Mapping
    glEnable(GL_AUTO_NORMAL);
    glEnable(GL_NORMALIZE);
    glEnable(GL_DEPTH_TEST); // Enables Depth Testing
    glDepthFunc(GL_LESS);
    glShadeModel(GL_SMOOTH); // Enables Smooth Color Shading
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void GLWidget::paintGL()
{
    glDrawBuffer(GL_BACK);
    glClearColor(0.6f, 0.8f, 1.0f, 0.0f);

    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

   /*Angulo	ratio		znear, zfar*/


   gluPerspective(50.0, width / height, 1.0, 50000.0);

   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();

   /*pos cam		pto central	vector up*/
   gluLookAt(this->glcam_pos_X, this->glcam_pos_Y, this->glcam_pos_Z,
             this->glcam_foa_X, this->glcam_foa_Y, this->glcam_foa_Z,
             0., 0., 1.);

    // absolute axis
    glLineWidth(3.0f);
    glColor3f(0.7, 0., 0.);
    glBegin(GL_LINES);
    v3f(0.0, 0.0, 0.0);
    v3f(10.0, 0.0, 0.0);
    glEnd();
    glColor3f(0., 0.7, 0.);
    glBegin(GL_LINES);
    v3f(0.0, 0.0, 0.0);
    v3f(0.0, 10.0, 0.0);
    glEnd();
    glColor3f(0., 0., 0.7);
    glBegin(GL_LINES);
    v3f(0.0, 0.0, 0.0);
    v3f(0.0, 0.0, 10.0);
    glEnd();
    glLineWidth(1.0f);

    //SUELO
    glColor3f(0.6, 0.6, 0.6);
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    for (int i = 0; i < ((int) MAXWORLD + 1); i++) {
        v3f(-(int) MAXWORLD * 10 / 2. + (float) i * 10, -(int) MAXWORLD * 10 / 2., 0.);
        v3f(-(int) MAXWORLD * 10 / 2. + (float) i * 10, (int) MAXWORLD * 10 / 2., 0.);
        v3f(-(int) MAXWORLD * 10 / 2., -(int) MAXWORLD * 10 / 2. + (float) i * 10, 0.);
        v3f((int) MAXWORLD * 10 / 2., -(int) MAXWORLD * 10 / 2. + (float) i * 10, 0.);
    }
    glEnd();

    UpdateRobot();

    //robot
    drawRobot();

    RunGraphicsAlgorithm();

}

void GLWidget::resizeGL(int width, int height)
{
    glViewport(0, 0, (GLint) width, (GLint) height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-0.5, +0.5, -0.5, +0.5, 4.0, 15.0);
    glMatrixMode(GL_MODELVIEW);
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{

    float desp = 0.01;
    float x = event->x();
    float y = event->y();

    if ((x - old_x) > 0.0) longi -= desp;
    else if ((x - old_x) < 0.0) longi += desp;

    if ((y - old_y) > 0.0) lati += desp;
    else if ((y - old_y) < 0.0) lati -= desp;

    if(event->buttons()==Qt::LeftButton ){

        this->glcam_pos_X = radius * cosf(lati) * cosf(longi) + this->glcam_foa_X;
        this->glcam_pos_Y = radius * cosf(lati) * sinf(longi) + this->glcam_foa_Y;
        this->glcam_pos_Z = radius * sinf(lati) + this->glcam_foa_Z;
    }

    if(event->buttons() == Qt::RightButton){

        this->glcam_foa_X = -radius * cosf(lati) * cosf(longi) + this->glcam_pos_X;
        this->glcam_foa_Y = -radius * cosf(lati) * sinf(longi) + this->glcam_pos_Y;
        this->glcam_foa_Z = -radius * sinf(lati) + this->glcam_pos_Z;

    }


    old_x = x;
    old_y = y;
}

void GLWidget::wheelEvent(QWheelEvent* event)
{
    float vx, vy, vz;

    vx = (this->glcam_foa_X - this->glcam_pos_X) / radius;
    vy = (this->glcam_foa_Y - this->glcam_pos_Y) / radius;
    vz = (this->glcam_foa_Z - this->glcam_pos_Z) / radius;

    if (event->delta() > 0) {
        this->glcam_foa_X = this->glcam_foa_X + vx;
        this->glcam_foa_Y = this->glcam_foa_Y + vy;
        this->glcam_foa_Z = this->glcam_foa_Z + vz;

        this->glcam_pos_X = this->glcam_pos_X + vx;
        this->glcam_pos_Y = this->glcam_pos_Y + vy;
        this->glcam_pos_Z = this->glcam_pos_Z + vz;
    }

    if (event->delta() < 0) {
        this->glcam_foa_X = this->glcam_foa_X - vx;
        this->glcam_foa_Y = this->glcam_foa_Y - vy;
        this->glcam_foa_Z = this->glcam_foa_Z - vz;

        this->glcam_pos_X = this->glcam_pos_X - vx;
        this->glcam_pos_Y = this->glcam_pos_Y - vy;
        this->glcam_pos_Z = this->glcam_pos_Z - vz;
    }

}

void GLWidget::setToCamera1 () {
    this->glcam_pos_X=0.;
    this->glcam_pos_Y=(int)MAXWORLD*10/1.2;
    this->glcam_pos_Z=150.;

    this->glcam_foa_X=0.;
    this->glcam_foa_Y=0.;
    this->glcam_foa_Z=0.;

//    lati = asin( (this->glcam_pos_Z -this->glcam_foa_Z) /radius);
//    longi = acos ( (this->glcam_pos_Y - this->glcam_foa_Y)/ (radius* sinf(lati)) );

}
/*
void GLWidget::setToCamera2 () {
    this->glcam_pos_X=-(int)MAXWORLD*10/1.2;
    this->glcam_pos_Y=0.;
    this->glcam_pos_Z=150.;

    this->glcam_foa_X=0.;
    this->glcam_foa_Y=0.;
    this->glcam_foa_Z=0.;
}

void GLWidget::setToCamera3 () {
    this->glcam_pos_X=0.;
    this->glcam_pos_Y=-(int)MAXWORLD*10/1.2;
    this->glcam_pos_Z=150.;

    this->glcam_foa_X=0.;
    this->glcam_foa_Y=0.;
    this->glcam_foa_Z=0.;
}

void GLWidget::setToCamera4 () {
    this->glcam_pos_X=(int)MAXWORLD*10/1.2;
    this->glcam_pos_Y=0.;
    this->glcam_pos_Z=150.;

    this->glcam_foa_X=0.;
    this->glcam_foa_Y=0.;
    this->glcam_foa_Z=0.;
}
*/

void GLWidget::UpdateRobot()
{
    mutex.lock();

    this->robot_x = this->robot->getSensors()->getRobotPoseX()/100;
    this->robot_y = this->robot->getSensors()->getRobotPoseY()/100;
    this->robot_theta = this->robot->getSensors()->getRobotPoseTheta();
    this->rueda_theta = -this->robot->getActuators()->getMotorW();
    this->laserData = this->robot->getSensors()->getLaserData();

    mutex.unlock();
}



void GLWidget::drawRobot()
{
    //////////////////////////// Pioneer //////////////////////////////

    mutex.lock();
    // Robot Frame of Reference
    float posx = this->robot_x / 100.;
    float posy = this->robot_y / 100.;
    float posz = 0.;
    float foax = this->robot_x / 100.;
    float foay = this->robot_y / 100.;
    float foaz = 10.;
    float roll = this->robot_theta;

    glTranslatef(posx, posy, posz);
    float dx = (foax - posx);
    float dy = (foay - posy);
    float dz = (foaz - posz);
    float longiPioneer = (float) atan2(dy, dx)*360. / (2. * PI);
    glRotatef(longiPioneer, 0., 0., 1.);
    float r = sqrt(dx * dx + dy * dy + dz * dz);
    float latiPioneer;
    if (r < 0.00001) latiPioneer = 0.;
    else latiPioneer = acos(dz / r)*360. / (2. * PI);
    glRotatef(latiPioneer, 0., 1., 0.);
    glRotatef(roll, 0., 0., 1.);

    // X axis
    glColor3f(1., 0., 0.);
    glBegin(GL_LINES);
    v3f(0.0, 0.0, 0.0);
    v3f(5.0, 0.0, 0.0);
    glEnd();

    // Y axis
    glColor3f(0., 1., 0.);
    glBegin(GL_LINES);
    v3f(0.0, 0.0, 0.0);
    v3f(0.0, 5.0, 0.0);
    glEnd();

    // Z axis
    glColor3f(0., 0., 1.);
    glBegin(GL_LINES);
    v3f(0.0, 0.0, 0.0);
    v3f(0.0, 0.0, 5.0);
    glEnd();

    // robot body
    glEnable(GL_LIGHTING);
    glPushMatrix();
    glTranslatef(1., 0., 0.);

    // the body it is not centered. With this translation we center it
    loadModel(); // CARGAMOS EL MODELO DEL PIONEER
    glPopMatrix();
    glDisable(GL_LIGHTING);

    //sensors
    drawLaser();

    mutex.unlock();
}

void GLWidget::drawLaser()
{

    float Xp_sensor;
    float Yp_sensor;

    ///LASER
    float matColors[4];
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING); // luces... entramos en parte de dibujado del pioneer con texturas
    matColors[0] = 1.0;
    matColors[1] = 0.0;
    matColors[2] = 0.0;
    matColors[3] = 0.5;
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, matColors);
    glBegin(GL_POLYGON);

    v3f(0, 0,  2);

    for (unsigned k = 0; k < laserData.size(); k++) {

        Xp_sensor = this->laserData[k]/100 * cos(((float) k - 90.) * 3.14/180);
        Yp_sensor = this->laserData[k]/100 * sin(((float) k - 90.) * 3.14/180);

        v3f(Xp_sensor, Yp_sensor,  2.0f);

    }
    v3f(0, 0,  2);

    glEnd();
    glDisable(GL_LIGHTING);

}
