#pragma once
#include "Arduino.h"
#include "math.h"
/*
Library for converting Gauss krüger coordinates into WGS 84 coordinates.
Library by Jonas Woerner (jonas.woerner@online.de)
based on the work of Ingo Steiner.
*/
class GK_in_WGS84
{
public: double Latitude;
		double Longitude;
		double Altitude;
private:
	int Bezugsmeridian;
	double HW;
	double RW;
	double Height;
	String Meridian;
	String GebietHelmi;
	int Meridianneu;


	//Ellepsoid-Koordinaten auf dem Bessel Ellipsoid
	double B;
	double L;

	//Vektoren in DHDN/Bessel
	double xB;
	double yB;
	double zB;

	//Vektoren in WGS84
	double xW;
	double yW;
	double zW;


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
	
void Convert_to_WGS(double Hoch, double Rechts, double& lat, double& lon)
{
	HW = Hoch;
	RW = Rechts;
	Height = 400;
	Meridian = "Automatische Ermittlung";
	GebietHelmi = "Deutschland Potsdamm 2001";
	
	KonstanteParameter();

	GKnachBL();

	//Ellipsoid Vektoren in DHDN
	VektorenDNDH();

	//HelmertTransformation
	ParameterHelmert(GebietHelmi);
	Helmert();

	Vektorenumrechnung();
	lat = Latitude;
	lon = Longitude;
}

//public: void GK_in_WGS(double Hochwert, double Rechtswert, double Hoehe, String Bezugsmeridian, String HelmerttransformationsArt)
//{
//	HW = Hochwert;
//	RW = Rechtswert;
//	Height = Hoehe;
//	Meridian = Bezugsmeridian;
//	GebietHelmi = HelmerttransformationsArt;
//
//	KonstanteParameter();
//
//	GKnachBL();
//
//	//Ellipsoid Vektoren in DHDN
//	VektorenDNDH();
//
//	//HelmertTransformation
//	ParameterHelmert(GebietHelmi);
//	Helmert();
//
//	Vektorenumrechnung();
//}
		void KonstanteParameter()
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

		void GKnachBL()
		{
			MeridianUmrechnung();

			//Bessel Ellipsoid
			double n = (aB - bB) / (aB + bB);
			double alpha = (aB + bB) / 2.0 * (1.0 + 1.0 / 4.0 * n * n + 1.0 / 64.0 * pow(n, 4));
			double beta = 3.0 / 2.0 * n - 27.0 / 32.0 * pow(n, 3) + 269.0 / 512.0 * pow(n, 5);
			double gamma = 21.0 / 16.0 * n * n - 55.0 / 32.0 * pow(n, 4);
			double delta = 151.0 / 96.0 * pow(n, 3) - 417.0 / 128.0 * pow(n, 5);
			double epsilon = 1097.0 / 512.0 * pow(n, 4);

			double y0 = Meridianneu / 3.0;
			double y = RW - y0 * 1000000 - 500000;

			double Bnull = HW / alpha;
			double Bf = Bnull + beta * sin(2 * Bnull) + gamma * sin(4 * Bnull) + delta * sin(6 * Bnull) + epsilon * sin(8 * Bnull);

			double Nf = aB / sqrt(1.0 - e2B * pow(sin(Bf), 2));
			double ETAf = sqrt((aB * aB) / (bB * bB) * e2B * pow(cos(Bf), 2));
			double tf = tan(Bf);

			double b1 = tf / 2.0 / (Nf * Nf) * (-1.0 - (ETAf * ETAf)) * (y * y);
			double b2 = tf / 24.0 / pow(Nf, 4) * (5.0 + 3.0 * (tf * tf) + 6.0 * (ETAf * ETAf) - 6.0 * (tf * tf) * (ETAf * ETAf) - 4.0 * pow(ETAf, 4) - 9.0 * (tf * tf) * pow(ETAf, 4)) * pow(y, 4);

			B = (Bf + b1 + b2) * 180 / PI;

			double l1 = 1.0 / Nf / cos(Bf) * y;
			double l2 = 1.0 / 6.0 / pow(Nf, 3) / cos(Bf) * (-1.0 - 2.0 * (tf * tf) - (ETAf * ETAf)) * pow(y, 3);

			L = Meridianneu + (l1 + l2) * 180 / PI;

		}

		void MeridianUmrechnung()
		{

			if (Meridian == "Automatische Ermittlung")
			{
				String sStr(RW);
				int Zone = atoi(sStr.substring(0, 1).c_str());

				//Serial.println(Zone);

				Meridianneu = Zone * 3;
			}
			else
			{
				Meridianneu = atoi(Meridian.c_str());
			}
		}

		void VektorenDNDH()
		{
			//Querkr?mmunsradius
			double N = aB / sqrt(1.0 - e2B * pow(sin(B / 180 * PI), 2));

			// Ergebnis Vektoren	
			xB = (N + Height) * cos(B / 180 * PI) * cos(L / 180 * PI);
			yB = (N + Height) * cos(B / 180 * PI) * sin(L / 180 * PI);
			zB = (N * (bB * bB) / (aB * aB) + Height) * sin(B / 180 * PI);
		}

		void ParameterHelmert(String LandesParameter)
		{
			if (LandesParameter == "Deutschland Potsdamm 2001")
			{
				dx = 598.1; //Translation in X
				dy = 73.7; //Translation in Y
				dz = 418.2; //Translation in Z
				ex = -0.202; //Drehwinkel in Bogensekunden un die x-Achse
				ey = -0.045; //Drehwinkel in Bogensekunden un die y-Achse
				ez = 2.455; //Drehwinkel in Bogensekunden un die z-Achse
				m = 6.7;  //Ma?stabsfaktor in ppm 
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

		void Helmert()
		{
			//Umrechnung der Drehwinkel in Bogenma?
			double exRad = (ex * PI / 180.0) / 3600.0;
			double eyRad = (ey * PI / 180.0) / 3600.0;
			double ezRad = (ez * PI / 180.0) / 3600.0;

			//Ma?stabsumrechnung
			double mEXP = 1 - m * pow(10, -6);

			//Drehmatrix
			// 1         Ez    -Ez
			// -Ez       1      Ex 
			// Ey       -Ex     1

			//Rotierende Vektoren
			// = Drehmatrix * Vektoren in WGS84
			double RotVektor1 = 1.0 * xB + ezRad * yB + (-1.0 * eyRad * zB);
			double RotVektor2 = (-1.0 * ezRad) * xB + 1 * yB + exRad * zB;
			double RotVektor3 = (eyRad)* xB + (-1.0 * exRad) * yB + 1 * zB;

			//Ma?stab ber?cksichtigen
			double RotVectorM1 = RotVektor1 * mEXP;
			double RotVectorM2 = RotVektor2 * mEXP;
			double RotVectorM3 = RotVektor3 * mEXP;

			//Translation anbringen
			//dxT = Drehmatrix * dx * m
			double dxT = 1.0 * dx * mEXP + ezRad * dy * mEXP + (-1.0 * eyRad) * dz * mEXP;
			double dyT = (-1.0 * ezRad) * dx * mEXP + 1.0 * dy * mEXP + exRad * dz * mEXP;
			double dzT = (eyRad)* dx * mEXP + (-1.0 * exRad) * dy * mEXP + 1 * dz * mEXP;

			//Vektoren jetzt in WGS84
			xW = RotVectorM1 + dxT;
			yW = RotVectorM2 + dyT;
			zW = RotVectorM3 + dzT;
		}

		void Vektorenumrechnung()
		{
			double s = sqrt(xW * xW + yW * yW);
			double T = atan(zW * aW / (s * bW));
			double Bz = atan((zW + e2W * (aW * aW) / bW * pow(sin(T), 3)) / (s - e2W * aW * pow(cos(T), 3)));

			double Lz = atan(yW / xW);
			double N = aW / sqrt(1 - e2W * pow(sin(Bz), 2));

			Altitude = s / cos(Bz);
			Latitude = Bz * 180 / PI;
			Longitude = Lz * 180 / PI;
			Bezugsmeridian = Meridianneu;

		}
};
