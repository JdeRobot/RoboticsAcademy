#include "actuators.h"

Actuators::Actuators(Ice::CommunicatorPtr ic)
{
    this->ic = ic;

    // Contact to MOTORS interface
    Ice::ObjectPrx baseMotors = ic->propertyToProxy("introrob.Motors.Proxy");
    if (0 == baseMotors){
        motorsON = false;
		std::cout << "Motors configuration not specified" <<std::endl;

        //throw "Could not create proxy with motors";
	}else{
		// Cast to motors
		try{
			mprx = jderobot::MotorsPrx::checkedCast(baseMotors);
			if (0 == mprx)
				throw "Invalid proxy introrob.Motors.Proxy";

			motorsON = true;
			std::cout << "Motors connected" << std::endl;
		}catch (Ice::ConnectionRefusedException& e){
			motorsON=false;
			std::cout << "Motors inactive" << std::endl;
		}
	}
    
    motorVout= 0;
    motorWout = 0;
    motorLout= 0;

}

void Actuators::update()
{
    if (motorsON) {
	    mutex.lock();

	    motorVin = this->mprx->getV();
	    motorWin = this->mprx->getW();
	    motorLin = this->mprx->getL();

	    mutex.unlock();
    }
}

void Actuators::setActuators()
{
	if (motorsON) {
		mutex.lock();

		if (motorWout < 5 && motorWout>-5)
		    this->mprx->setW(0.);

		this->mprx->setW(motorWout);
		this->mprx->setL(motorLout);
		this->mprx->setV(motorVout);

		mutex.unlock();
	}
}


///////////////// GETTER //////////////////
float Actuators::getMotorV()
{

	float v;
	mutex.lock();
	if (motorsON)
		v = motorVin;
	else
		v = 0;
	mutex.unlock();

    return v;
	
}

float Actuators::getMotorW()
{
	float w;
	mutex.lock();
	if (motorsON)
		w = motorVin;
	else
		w = 0;
	mutex.unlock();

    return w;
}

float Actuators::getMotorL()
{
    float l;
	mutex.lock();
	if (motorsON)
		l = motorVin;
	else
		l = 0;
	mutex.unlock();

    return l;
}

///////////////// SETTER //////////////////
void Actuators::setMotorSTOP()
{
	if (motorsON) {
		mutex.lock();
		this->motorVout = 0;
		this->motorWout = 0;
		this->motorLout = 0;
		mutex.unlock();
	}
}

void Actuators::setMotorV(float motorV)
{
	if (motorsON) {
		mutex.lock();
		this->motorVout = motorV;
		mutex.unlock();
	}
}

void Actuators::setMotorW(float motorW)
{
	if (motorsON) {
		mutex.lock();
		this->motorWout = motorW;
		mutex.unlock();
	}
}

void Actuators::setMotorL(float motorL)
{
	if (motorsON) {
		mutex.lock();
		this->motorLout = motorL;
		mutex.unlock();
	}

}

