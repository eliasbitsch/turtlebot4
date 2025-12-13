// --- Philip Stix ---

#include <iostream>
#include <vector>
#include <cmath>

const int GRID_WIDTH = 20;
const int GRID_HEIGHT = 10;

// Konvertiere echte Koordinaten ins Grid
void toGrid(double x, double y, int &gx, int &gy) {
    gx = std::round(x);
    gy = std::round(y);
    if (gx < 0) gx = 0;
    if (gx >= GRID_WIDTH) gx = GRID_WIDTH - 1;
    if (gy < 0) gy = 0;
    if (gy >= GRID_HEIGHT) gy = GRID_HEIGHT - 1;
}

// Grid drucken
void printGrid(const std::vector<std::vector<char>> &grid) {
    for (int y = GRID_HEIGHT - 1; y >= 0; --y) {
        for (int x = 0; x < GRID_WIDTH; ++x) {
            std::cout << grid[y][x];
        }
        std::cout << "\n";
    }
    std::cout << "\n";
}

// Grid initialisieren (leer)
std::vector<std::vector<char>> createGrid() {
    return std::vector<std::vector<char>>(GRID_HEIGHT, std::vector<char>(GRID_WIDTH, '.'));
}

// Markiere einen Punkt auf dem Grid
void markPosition(std::vector<std::vector<char>> &grid, double x, double y) {
    int gx, gy;
    toGrid(x, y, gx, gy);
    grid[gy][gx] = '*';
}


// Beispiel Main
int main() {
    // Grid erstellen
    std::vector<std::vector<char>> grid = createGrid();

    // Beispiel-Zielpunkt
    double example_target_x = 15.0;
    double example_target_y = 7.0;

    markPosition(grid, example_target_x, example_target_y);

    // Grid anzeigen
    printGrid(grid);

    return 0;
}
