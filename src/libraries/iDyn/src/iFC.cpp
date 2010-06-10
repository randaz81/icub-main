/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Matteo Fumagalli
* email:   matteo.fumagalli@iit.it
* website: www.robotcub.org
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

#include <iostream>
#include <iomanip>
#include <iCub/iDyn/iFC.h>

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iDyn;

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
iFTransform::iFTransform()
{
	initFTransform();
}
iFTransform::iFTransform(const Matrix &_R, double _x, double _y, double _z) 
{
	initFTransform();
	setP(_x,_y,_z);
	setR(_R);
	setH(R,p);
}
iFTransform::iFTransform(const Matrix &_R, const Vector &_p)
{
	initFTransform();
	setP(_p);
	setR(_R);
	setH(R,p);
}
void iFTransform::initFTransform()
{
	R.resize(3,3);
	H.resize(4,4);
	p.resize(3);
	FT.resize(6);
	p=0.0;
	R=eye(3,3);
	H=eye(4,4);
	FT=0.0;
}
void iFTransform::setR(const Matrix &_R)
{
	R=_R;
}
void iFTransform::setP(double _x, double _y, double _z)
{
	p(0)=_x;
	p(1)=_y;
	p(2)=_z;
}
void iFTransform::setP(const Vector &_p)
{
	p=_p;
}

void iFTransform::setPRH(const Matrix &_H)
{
	for(int i=0;i<3;i++)
		p(i)=_H(i,3);
	R=_H.submatrix(0,2,0,2);
	setH(_H);
}
void iFTransform::setPRH()
{
	for(int i=0;i<3;i++)
		p(i)=H(i,3);
	R=H.submatrix(0,2,0,2);
}
void iFTransform::setPRH(const Matrix &_R,const Vector &_p)
{
	setH(_R,_p);
	setPRH(H);
}
void iFTransform::setH(const Matrix &_R, double _x, double _y, double _z)
{
	H=eye(4,4);
	for(int i=0; i<3; i++)
		for(int j=0; j<3; j++)
			H(i,j)=_R(i,j);
	H(0,3)=_x;
	H(1,3)=_y;
	H(2,3)=_z;
}
void iFTransform::setH(const Matrix &_R, const Vector &_p)
{
	H=eye(4,4);
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			H(i,j)=_R(i,j);
		}
		H(i,3)=_p(i);
	}
}
void iFTransform::setH(const Matrix &_H)
{
	R=_H.submatrix(0,2,0,2);
	for(int i=0;i<3;i++)
		p(i)=_H(i,3);
	H=_H;
}
yarp::sig::Vector iFTransform::setFT(const Vector &_FT)
{
		return FT=_FT;
}

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
// Definition of iSFrame

iSFrame::iSFrame()
{
	l=0;
	initSFrame();
}

iSFrame::iSFrame(int _l)
{
	l=_l;
	initSFrame();	
}
void iSFrame::initSFrame()
{
	H.resize(4,4);
	FT.resize(6);
	H=0.0;
	FT=0.0;
	Sensore=0;
	Limb=0;
	Link=new iFTransform();
}

void iSFrame::setLink(int _l)
{
	l=_l;
}
void iSFrame::setFT(const yarp::sig::Vector &_FT)
{
	FT=Sensore->setFT(_FT);
}

void iSFrame::setSensorKin(int _l)
{
	Link->setPRH(Limb->getH(_l));
	H=Link->getH()*Sensore->getH();
}

void iSFrame::setSensorKin()
{
	Link->setPRH(Limb->getH(l));
	H=Link->getH()*Sensore->getH();
}

void iSFrame::setSensorKin(const Matrix &_H)
{
	Link->setPRH(_H);
	H=_H*Sensore->getH();
}

void iSFrame::setSensor(int _l, const yarp::sig::Vector &_FT)
{
	setSensorKin(_l);
	setFT(_FT);
}

void iSFrame::setSensor(const yarp::sig::Matrix &_H, const yarp::sig::Vector &_FT)
{
	setSensorKin(_H);
	setFT(_FT);
}

void iSFrame::setSensor(const yarp::sig::Vector &_FT)
{
	setSensorKin();
	setFT(_FT);
}

Vector iSFrame::getFT()
{
	return FT;
}
Matrix iSFrame::getH()
{
	return H;
}

void iSFrame::attach(iKin::iKinChain *_Limb)
{
	Limb=_Limb;
}

void iSFrame::attach(iFTransform *_Sensore)
{
		Sensore = _Sensore;
}

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////


iFB::iFB()
{
	Sensore=new iSFrame();
	EndEffector=new iFTransform();
	Limb= new iKin::iKinChain();
	initiFB();
}
iFB::iFB(int _l)
{
	l=_l;
	Sensore = new iSFrame(l);
	Limb= new iKin::iKinChain();
	EndEffector=new iFTransform();
	initiFB();
	//Sensore->setLink(l);
}
void iFB::attach(iKin::iKinChain *_Limb)
{
	Limb=_Limb;
	Sensore->attach(_Limb);
}
void iFB::attach(iFTransform *_sensore)
{
	Sensore->attach(_sensore);
}
void iFB::initiFB()
{
	//Sensore=0;
	//Limb=0;
	//EndEffector=0;
	l=0;
	Hs.resize(4,4);
	Hs=0.0;
	Fs.resize(6);
	Fs=0.0;
	He.resize(4,4);
	He=0.0;
	Fe.resize(6);
	Fe=0.0;
	Tse.resize(6,6);
	Tse=0.0;
	Teb.resize(6,6);
	Teb=0.0;

	d.resize(3);
	d=0.0;
	S.resize(3,3);
	S=0.0;
	R.resize(3,3);
	R=0.0;
}
void iFB::setLink(int _l)
{
	Sensore->setLink(_l);
	l=_l;
}
void iFB::setSensor(const Vector &_FT)
{
	Fs=_FT;
	Sensore->setSensor(_FT);
	Hs=Sensore->getH();
}
void iFB::setSensor(int _l, const Vector &_FT)
{
	Fs=_FT;
	l=_l;
	Sensore->setSensor(_l, _FT);
	Hs=Sensore->getH();
}
void iFB::setSensor(const Matrix &_H, const Vector &_FT)
{
	Fs=_FT;
	Sensore->setSensor(_H, _FT);
	Hs=Sensore->getH();
}
void iFB::setHe()
{
	EndEffector->setH(Limb->getH());
	He=EndEffector->getH();
}
void iFB::setHe(int _l)
{
	EndEffector->setH(Limb->getH(_l));
	He=EndEffector->getH();
}
void iFB::setHe(const Matrix &_H)
{
	EndEffector->setH(_H);
	He=EndEffector->getH();
}
void iFB::setTeb()
{
	setHe();
	for(int i=0;i<3;i++)
	{
		for(int j=0; j<3; j++)
		{
			Teb(i,j)=He(i,j);
			Teb(i+3,j+3)=He(i,j);
		}
	}
}
yarp::sig::Vector iFB::getFB(const Vector &_FT)
{
	setSensor(_FT);
	setTeb();
	setTse();
	setFe();
	return Teb*Fe;
}
yarp::sig::Vector iFB::getFB()
{
	setTeb();
	setTse();
	setFe();
	return Teb*Fe;
}


yarp::sig::Vector iFB::setFe()
{
	return Fe=Tse*Fs;
}
yarp::sig::Vector iFB::getFe()
{
	return Fe;
}
void iFB::setTse()
{	
	S=0.0;
	Tse=0.0;
	R=0.0;
	for(int i=0; i<3;i++)
	{
		d(i)=(Hs(i,3)-He(i,3));
	}
	d=He.submatrix(0,2,0,2).transposed()*d;
	S(0,1)=-d(2);
	S(0,2)=d(1);
	S(1,0)=d(2);
	S(1,2)=-d(0);
	S(2,0)=-d(1);
	S(2,1)=d(0);

	R=He.submatrix(0,2,0,2).transposed()*Hs.submatrix(0,2,0,2);
	S=S*R;

	for(int i=0; i<3;i++)
	{
		for(int j=0; j<3;j++)
		{
			Tse(i,j)=R(i,j);
			Tse(i+3,j+3)=R(i,j);
			Tse(i+3,j)=S(i,j);
		}
		
	}

}
