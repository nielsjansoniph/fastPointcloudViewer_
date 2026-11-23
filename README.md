# Fast Point Cloud Viewer (fpcv)

Schneller First-Person-Viewer für große Punktwolken (`.ply`, `.pcd`) auf Basis von OpenGL 3.3,
Dear ImGui und PCL. Mit verschiedenen Darstellungs-Shadern, frei drehbarer Modellausrichtung
und Kamerafahrten, die als Video aufgenommen werden können.

## Bauen

### Linux (Ubuntu 22.04)

Benötigte Pakete (einmalig):

```bash
sudo apt install build-essential cmake git pkg-config \
    libpcl-dev libopencv-dev libassimp-dev libglm-dev \
    libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
    libwayland-dev libxkbcommon-dev libgl-dev libdbus-1-dev python3
```

GLFW, glad, Dear ImGui und nativefiledialog-extended lädt CMake beim ersten Konfigurieren selbst herunter.

```bash
cmake -S . -B build/linux -DCMAKE_BUILD_TYPE=Release
cmake --build build/linux -j
```

### Windows

Abhängigkeiten über vcpkg (octomap, pcl, glad, glfw3, imgui, assimp, nfd, opencv), dann mit
CMake bauen, z. B. in VS Code. Ausgabe nach `build/Debug`.

## Starten

Das Programm lädt die Shader relativ zum Arbeitsverzeichnis (`../../`). Deshalb immer aus dem
Build-Ordner starten:

```bash
cd build/linux
./fpcv                          # leer starten, Wolken über „Load“ laden
./fpcv ../../scan.ply andere.pcd  # Wolken direkt beim Start laden
./fpcv --help
```

## Bedienung

| Eingabe | Funktion |
|---|---|
| W / A / S / D | vorwärts / links / rückwärts / rechts |
| R / F | hoch / runter (entlang Z) |
| Pfeiltasten | umsehen (drehen / hoch- und runterschauen) |
| linke Maustaste halten | umsehen mit der Maus |
| C | aktuelle Kameraposition als Wegpunkt aufnehmen (gedrückt halten + bewegen nimmt fortlaufend auf) |

Die Tastatur wirkt nur, solange der Mauszeiger nicht über dem Menü steht.

### Menü „Viewer settings“

- **Near / Far**: Stärke der Abdunklung mit der Entfernung (wenn „Use depth for point brightness“ aktiv).
- **Speed (m/s)**: Bewegungsgeschwindigkeit; −/+ ändert um Faktor 1,25.
- **Orientation**: Koordinatenkreuz ein/aus und Länge; Modell in 90°-Schritten um X/Y/Z drehen.
  Z zeigt nach oben. Die Drehung wird nicht gespeichert.
- **Pro Wolke**: PointSize und Shader-Auswahl:
  - *Default*: Farbe nach Entfernung
  - *Monocolor*: einheitliche Farbe
  - *Cube*: jeder Punkt als Würfel, jede Seite eigene Farbe (+X rot, −X cyan, +Y grün, −Y magenta, +Z blau, −Z gelb); Kantenlänge = PointSize in cm
  - *Sphere*: jeder Punkt als Kugel mit denselben Richtungsfarben, stufenlos; Durchmesser = PointSize in cm
  - *ColorGradient*, *debug*: Farbverläufe nach Entfernung
  - *zMap*: Farbe nach Höhe (Z)
- **Replay / Wegpunkt-Tabelle**: aufgenommene Wegpunkte abfahren, optional als Schleife und als
  Video aufzeichnen (MJPG-`.avi` im Ordner `records/` des Projekts, der Ordner muss existieren);
  Wegpunkte auswählen, löschen, verschieben.
