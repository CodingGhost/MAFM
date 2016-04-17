#pragma once
#include "Arduino.h"
#include "math.h"
/*
Library for converting WGS 84 coordinates into Gauss krüger coordinates.
Library by Jonas Woerner (jonas.woerner@online.de)
based on the work of Ingo Steiner.
*/
class WGS84_in_GK
{
private:
	double Hochwert;
	double Rechtswert;
	double Hohe;
	int Bezugsmeridian;

	double Lat;
	double Lon;
	double Alt;
	String Meridian;
	String GebietHelmi;
	int Meridianneu;

	//Vektoren in WGS84
	double xW;
	double yW;
	double zW;

	//Vektoren in Bessel
	double xB;
	double yB;
	double zB;

	//Vektoren in ERTS89
	double B;
	double L;
	double H;


	//Konstanten

	//WGS84 Ellipsoid
	double aW;
	double bW;
	double e2W;
	double fW;

	//Bessel Ellipsoid
	double aB;
	double bB;
	double e2B;
	double fB;

	//Helmert-Parameter
	double dx;
	double dy;
	double dz;
	double ex;
	double ey;
	double ez;
	double m;


public: 
	
	void Convert_to_GK(double Latitude, double Longitude, double& R, double& H)
	{
		Lat = Latitude;
		Lon = Longitude;
		Alt = 500;
		Meridian = "Automatische Ermittlung";
		GebietHelmi = "Deutschland Potsdamm 2001";
		KonstanteParameter();

		//Ellipsoid Koordinaten auf dem WGS84 Ellipsoid
		EllipsoidKoordinaten();

		//7 Parameter Helmert Transformation (Datumstransformation von WGS nach Bessel)
		//Parameter f?r Deutschland und ?sterreich
		ParameterHelmert(GebietHelmi);
		Helmert();

		//Vektoren in ETRS89
		VektorenETRS();

		//Umrechung von ETRS89 Vektoren in Gau? Kr?ger
		UmrechungGK();
		R = Rechtswert;
		H = Hochwert;
	}

//public: void WGS_in_GK(double Latitude, double Longitude, double Altitude, String Bezugsmeridian, String HelmerttransformationsArt)
//{
//	Lat = Latitude;
//	Lon = Longitude;
//	Alt = Altitude;
//	Meridian = Bezugsmeridian;
//	GebietHelmi = HelmerttransformationsArt;
//
//	KonstanteParameter();
//
//	//Ellipsoid Koordinaten auf dem WGS84 Ellipsoid
//	EllipsoidKoordinaten();
//
//	//7 Parameter Helmert Transformation (Datumstransformation von WGS nach Bessel)
//	//Parameter f?r Deutschland und ?sterreich
//	ParameterHelmert(GebietHelmi);
//	Helmert();
//
//	//Vektoren in ETRS89
//	VektorenETRS();
//
//	//Umrechung von ETRS89 Vektoren in Gau? Kr?ger
//	UmrechungGK();
//}

private: void KonstanteParameter()
{
	//WGS84 Ellipsoid
	aW = 6378137; //gro?e Halbachse
	bW = 6356752.3141; //kleine Halbachse
	e2W = (pow(aW, 2) - pow(bW, 2)) / pow(aW, 2); //1.Numerische Exzentrit?t
	fW = (aW - bW) / aW; //Abplattung 1: fW

						 //Bessel Ellipsoid
	aB = 6377397.155;
	bB = 6356078.962;
	e2B = (aB * aB - bB * bB) / (aB * aB);
}

private: void ParameterHelmert(String LandesParameter)
{
	if (LandesParameter == "Deutschland Potsdamm 2001")
	{
		dx = -598.1; //Translation in X
		dy = -73.7; //Translation in Y
		dz = -418.2; //Translation in Z
		ex = 0.202; //Drehwinkel in Bogensekunden un die x-Achse
		ey = 0.045; //Drehwinkel in Bogensekunden un die y-Achse
		ez = -2.455; //Drehwinkel in Bogensekunden un die z-Achse
		m = -6.7;  //Ma?stabsfaktor in ppm 
	}
	else if (LandesParameter == "Österreich")
	{
		dx = -577.326; //Translation in X
		dy = -90.129; //Translation in Y
		dz = -463.919; //Translation in Z
		ex = 5.137; //Drehwinkel in Bogensekunden un die x-Achse
		ey = 1.474; //Drehwinkel in Bogensekunden un die y-Achse
		ez = 5.297; //Drehwinkel in Bogensekunden un die z-Achse
		m = -2.423;  //Ma?stabsfaktor  in ppm
	}

}

private: void EllipsoidKoordinaten()
{
	//Querkr?mmunsradius
	double N = aW / sqrt(1 - e2W * pow(sin(Lat / 180 * PI), 2));

	// Ergebnis Vektoren
	xW = (N + Alt) * cos(Lat / 180 * PI) * cos(Lon / 180 * PI);
	yW = (N + Alt) * cos(Lat / 180 * PI) * sin(Lon / 180 * PI);
	zW = (N * bW * bW / (aW * aW) + Alt) * sin(Lat / 180 * PI);
}

private: void Helmert()
{
	//Umrechnung der Drehwinkel in Bogenma?
	double exRad = (ex * PI / 180.0) / 3600.0;
	double eyRad = (ey * PI / 180.0) / 3600.0;
	double ezRad = (ez * PI / 180.0) / 3600.0;

	//Ma?stabsumrechnung
	double mEXP = m * pow(10, -6) + 1;

	//Drehmatrix
	// 1         Ez    -Ez
	// -Ez       1      Ex 
	// Ey       -Ex     1

	//Rotierende Vektoren
	// = Drehmatrix * Vektoren in WGS84
	double RotVektor1 = 1 * xW + ezRad * yW + +(-1.0 * eyRad * zW);
	double RotVektor2 = (-1.0 * ezRad) * xW + 1 * yW + exRad * zW;
	double RotVektor3 = (eyRad)* xW + (-1.0 * exRad) * yW + 1 * zW;

	//Ma?stab ber?cksichtigen
	double RotVectorM1 = RotVektor1 * mEXP;
	double RotVectorM2 = RotVektor2 * mEXP;
	double RotVectorM3 = RotVektor3 * mEXP;

	//Translation anbringen
	//dxT = Drehmatrix * dx * m
	double dxT = 1 * dx * mEXP + ezRad * dy * mEXP + (-1.0 * eyRad) * dz * mEXP;
	double dyT = (-1.0 * ezRad) * dx * mEXP + 1 * dy * mEXP + exRad * dz * mEXP;
	double dzT = (eyRad)* dx * mEXP + (-1.0 * exRad) * dy * mEXP + 1 * dz * mEXP;

	//Vektoren jetzt in DHDN/Bessel
	xB = RotVectorM1 + dxT;
	yB = RotVectorM2 + dyT;
	zB = RotVectorM3 + dzT;
}

private: void VektorenETRS()
{
	double s = sqrt(xB * xB + yB * yB);
	double T = atan(zB * aB / (s * bB));
	double BR = atan((zB + e2B * aB * aB / bB * pow(sin(T), 3)) / (s - e2B * aB * pow(cos(T), 3)));
	double LR = atan(yB / xB);
	double N = aB / sqrt(1 - e2B * pow(sin(BR), 2));
	double HR = s / cos(BR) - N;

	B = BR * 180 / PI;
	L = LR * 180 / PI;
	H = HR;
	Hohe = H;
}
private: void MeridianUmrechnung()
{

	if (Meridian == "Automatische Ermittlung")
	{
		if (abs(L) - 6 < 1.5)
		{
			Meridianneu = 6;
		}
		else if (abs(L) - 9 < 1.5)
		{
			Meridianneu = 9;
		}
		else if (abs(L) - 12 < 1.5)
		{
			Meridianneu = 12;
		}
		else
		{
			Meridianneu = 15;
		}
	}
	else
	{
		Meridianneu = atoi(Meridian.c_str());
	}
}

private: void UmrechungGK()
{
	MeridianUmrechnung();

	//Bessel-Ellipsoid 
	double n = (aB - bB) / (aB + bB);
	double alpha = (aB + bB) / 2.0 * (1.0 + 1.0 / 4.0 * pow(n, 2) + 1.0 / 64.0 * pow(n, 4));

	double beta = -3.0 / 2.0 * n + 9.0 / 16.0 * pow(n, 3) - 3.0 / 32.0 * pow(n, 5);
	double gamma = 15.0 / 16.0 * (n * n) - 15.0 / 32.0 * pow(n, 4);
	double delta = -35.0 / 48.0 * pow(n, 3) + 105.0 / 256.0 * pow(n, 5);
	double epsilon = 315.0 / 512.0 * pow(n, 4);

	double LL = (L - Meridianneu) * (PI / 180);
	double BB = (B / 180) * PI;

	double NN = aB / sqrt(1 - e2B * pow(sin(BB), 2));
	double Nu = sqrt((aB * aB) / (bB * bB) * e2B * pow(cos(BB), 2));
	double t = tan(BB);
	//DU GEILHEIT VON PROGRAMM ICH LIEBE DICH
	double Bogenlaenge = alpha * (BB + beta * sin(2 * BB) + gamma * sin(4 * BB) + delta * sin(6 * BB) + epsilon * sin(8 * BB));

	double h1 = t / 2 * NN * pow(cos(BB), 2) * LL * LL;
	double h2 = t / 24 * NN * pow(cos(BB), 4) * (5 - t * t + 9 * Nu * Nu + 4 * pow(Nu, 4)) * pow(LL, 4);
	Hochwert = Bogenlaenge + h1 + h2;
	double r1 = NN * cos(BB) * LL;
	double r2 = NN / 6 * pow(cos(BB), 3) * (1 - t * t + Nu * Nu) * pow(LL, 3);
	Rechtswert = r1 + r2 + 500000 + Meridianneu / 3 * 1000000;

	Bezugsmeridian = Meridianneu;

}
};