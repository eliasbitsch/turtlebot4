#include <fstream>
#include <cmath>
#include <iostream>
#include <limits>

struct Rect 
{
    double x0,y0,x1,y1; // Definiert die koordinaten aller 4 Eckpunkte
};

bool intersect(double rx, double ry, double ang, // Überprüfung ob Lidar Strahl ein hindernis (Rechteck r) schneidet
               Rect r, double &dist) // Strahl aus (rx,ry) mit Richtung ang
{
    double dx = cos(ang);//Strahl wird definiert P(t)=(rx,ry)+t⋅(dx,dy)
    double dy = sin(ang);

    dist = std::numeric_limits<double>::infinity(); // Distanz wird initialisert

    // jede Rechteckkante testen
    double xs[4][4] = {
        {r.x0,r.y0,r.x1,r.y0}, // Hier sind die Koordinaten aller vier Rechteckkanten aufgelistet
        {r.x1,r.y0,r.x1,r.y1},
        {r.x1,r.y1,r.x0,r.y1},
        {r.x0,r.y1,r.x0,r.y0}
    };

    for(int i=0;i<4;i++) // For-Schleife Testet jede der vier Kanten Auf Treffer vom Laserstrahl
    {
        double x1=xs[i][0], y1=xs[i][1]; //Auslesen der Eckpunkte
        double x2=xs[i][2], y2=xs[i][3];

        double vx = x2-x1; // Berechnung des Richtungsvektors der Kante
        double vy = y2-y1;

        double det = dx*(-vy) - dy*(-vx);
        if(fabs(det) < 1e-6) continue;

        // Schnittpunkt berechnen Laserstrahl/Kante

        double t = ((x1-rx)*(-vy) - (y1-ry)*(-vx)) / det; // Abstand entlang des Lasers
        double u = ((x1-rx)*(-dy) - (y1-ry)*(-dx)) / det; // Position auf der Rechteckkante

        if(t > 0 && u >= 0 && u <= 1) // Überprüfung ob Treffer auf dem Segment liegt
            dist = std::min(dist, t); // Falls mehrere Kanten getroffen, wird kleinste Distanz gewählt
    }
    return std::isfinite(dist);
}

int main() 
{
    std::ofstream out("data/data.txt"); // Ausgabedatei wird geöffnet

    double rx=0, ry=0, yaw=0; // Sensorposition festlegen
    out << rx << " " << ry << " " << yaw << "\n";

    Rect rects[2] = { // Definition für zwei hindernisse
        { 2.0,  1.0,  3.5, 2.5},
        {-3.5, -2.0, -2.0, 1.0}
    };

    for(double a=-M_PI; a<M_PI; a+=0.01)  // 360 Grad Scan Schrittweite 0.1rad
    {
        double best = 10.0; 

        for(int i=0;i<2;i++)
        {
            double d;
            if(intersect(rx,ry,a,rects[i],d)) //Berechnung ob Strahl mit richtung a rects[i] trifft
                best = std::min(best, d); // Kleinste Entfernung
        }

        if(best < 10.0)
            out << a << " " << best << "\n";
        else
            out << a << " " << 10.0 << "\n";
    }

    out << "ENDSCAN\n";
    out.close();

    std::cout<<"core: 360deg scan generated\n";
    return 0;
}
