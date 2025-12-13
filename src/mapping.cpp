#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>

#define W 500 //Kartengröße definieren (Breite Höhe)
#define H 500
#define FREE 255 // Freie Zelle Weiß
#define OCC 0 // Hindernisse schwarz anzeigen
#define FILL 150 // freier Raum entlang vom Strahl grau färben

int main() {
    std::ifstream in("data/data.txt"); //scan Datei einlesen
    if(!in.is_open()) 
    {
        std::cerr << "cannot open data.txt\n";
        return 1;
    }

    double rx, ry, yaw; // Startposition und Orientierung des Roboters
    in >> rx >> ry >> yaw;

    std::vector<std::pair<double,double>> pts;

    while(true) 
    {
        std::string s;
        in >> s; // Bis zum nächsten leerzeichen oder Zeilenumbruch
        if(!in || s=="ENDSCAN") break;
        double a = std::stod(s), r; //Umwandlung von String zu Zahl
        in >> r;
        pts.push_back({ //Jede Messung wird in x und y umgerechnet
            rx + cos(yaw + a) * r,
            ry + sin(yaw + a) * r
        });
    }
    in.close();

    // min/max x und y Wert Laserpunkte
    double minx=rx, maxx=rx, miny=ry, maxy=ry; // Auf Sensorposition (0,0) setzen
    for(auto&p:pts)
    {
        minx=std::min(minx,p.first); // Kleinster x Wert alle Punkte
        maxx=std::max(maxx,p.first);
        miny=std::min(miny,p.second);
        maxy=std::max(maxy,p.second);
    }

    double scale = std::min( //Weltkoordinaten werden so skaliert das Roboter in 500x500 passt
        (W-1)/(maxx-minx+1e-6),
        (H-1)/(maxy-miny+1e-6)
    );

    int rpx = (rx-minx)*scale; // Position des Roboters im Raster
    int rpy = (ry-miny)*scale;

    std::vector<unsigned char> map(W*H, FREE); // Occupancy Grid wird angelegt

    // Ray-Fill
    for(auto&p:pts) 
    {
        int px = (p.first-minx)*scale; //Pixelkoordinaten Hindernisse
        int py = (p.second-miny)*scale;

        int dx = px - rpx; // Unterschied zwischen Roboterpixel und Hindernisspixel
        int dy = py - rpy;
        int steps = std::max(abs(dx),abs(dy)); 

        for(int i=0;i<steps;i++) // Freiraum des lasers wird markiert
        {
            int x = rpx + dx*i/steps;
            int y = rpy + dy*i/steps;

            if(x>=0 && x<W && y>=0 && y<H) 
                map[(H-y-1)*W+x] = FILL;
        }

        if(px>=0&&px<W&&py>=0&&py<H)
            map[(H-py-1)*W+px] = OCC; // Markiert die Hindernisse Schwarz
    }

    std::ofstream out("map.pgm");
    out<<"P2\n"<<W<<" "<<H<<"\n255\n"; // PGM Bild geneieren
    for(int i=0;i<W*H;i++) //Karte wird Zeile für zeile in die Datei geschrieben
    {
        out<<(int)map[i]<<" ";
        if(i%W==0) out<<"\n";
    }
    out.close();

    std::cout<<"mapping: filled obstacles generated\n"; // Ausgabe aufs Terminal
    return 0;
}
