# Fast Point Cloud Viewer (fpcv)

Schneller First-Person-Viewer für große Punktwolken (`.ply`, `.pcd`, Eckpunkte aus `.obj`) auf Basis von OpenGL 3.3,
Dear ImGui und PCL. Mit verschiedenen Darstellungs-Shadern, frei drehbarer Modellausrichtung
und Kamerafahrten, die als Video aufgenommen werden können.

## Bauen

### Linux (Ubuntu 22.04)

Benötigte Pakete (einmalig):

```bash
sudo apt install build-essential cmake git pkg-config \
    libpcl-dev libopencv-dev libglm-dev \
    libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
    libwayland-dev libxkbcommon-dev libgl-dev libdbus-1-dev python3
```

GLFW, glad, Dear ImGui und nativefiledialog-extended lädt CMake beim ersten Konfigurieren selbst herunter.

```bash
cmake -S . -B build/linux -DCMAKE_BUILD_TYPE=Release
cmake --build build/linux -j
```

### Windows

Abhängigkeiten über vcpkg (pcl, opencv, glad, glfw3, imgui, nfd), dann mit
CMake bauen, z. B. in VS Code. Ausgabe nach `build/Debug`.

## Starten

Das Programm kann aus jedem Ordner gestartet werden; die Shader findet es über den beim Bauen
einkompilierten Projektpfad (`shaders/`, abweichend per Umgebungsvariable `FPCV_SHADER_DIR`).

```bash
./build/linux/fpcv                           # leer starten, Wolken über „Load“ laden
./build/linux/fpcv scan.ply andere.pcd       # Wolken direkt beim Start laden
./build/linux/fpcv modell.obj                # bei .obj nur die Eckpunkte (mit Farben), automatisch von Y-oben auf Z-oben gedreht
./build/linux/fpcv --help
```

Große Dateien werden im Hintergrund geladen; ein Fenster zeigt den Fortschritt (bei binären
`.ply` und `.obj` in Prozent, bei `.pcd` und ASCII-`.ply` nur als laufender Balken) und kann
das Laden abbrechen. Mehrere Dateien werden nacheinander geladen.

Auf Laptops mit zwei Grafikkarten (NVIDIA Optimus/PRIME) nutzt das Programm automatisch die
NVIDIA-Karte – der Unterschied ist groß (102 Mio. Punkte: 2,4 statt 35 Bilder/s ohne LOD). Die
Konsole zeigt beim Start `OpenGL renderer: …`. Mit `FPCV_INTEGRATED_GPU=1` bleibt es bei der
integrierten Grafik.

## Projektstruktur

- `src/` – C++-Quellen (`main.cpp` Einstieg, `Viewer` Anwendung, `PointShaders` Tabelle aller Shader,
  `CameraPath` Kamerafahrten, `CloudLoader` Dateien lesen und in Blöcke sortieren, `Cloud`
  GPU-Daten, Culling und LOD, `Axes`, `Camera`, …)
- `shaders/` – GLSL-Shader; `common.glsl` enthält gemeinsame Uniforms und Funktionen
- `records/` – aufgenommene Videos

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
- **Level of detail**: Die Punkte sind beim Laden in räumliche Blöcke sortiert (Octree, höchstens
  65 536 Punkte je Block). Gezeichnet werden nur Blöcke im Sichtfeld; entfernte Blöcke zeigen nur
  einen Teil ihrer Punkte, nahe Blöcke alle.
  - *Level of detail*: aus = alle Punkte der sichtbaren Blöcke. Während einer Videoaufnahme ist
    LOD immer aus (volle Qualität, die Bildrate spielt dort keine Rolle).
  - *Point budget (M)*: höchstens so viele Millionen Punkte pro Bild. Reicht es nicht, werden
    zuerst die entfernten Blöcke weiter ausgedünnt und ihre Punkte etwas größer gezeichnet.
  - *Detail*: Punkte je Pixel der Blockgröße auf dem Bildschirm – höher = mehr Punkte in der Ferne.
  - *Drawn*: gezeichnete / geladene Punkte.
- **Orientation**: Koordinatenkreuz ein/aus und Länge; Modell in 90°-Schritten um X/Y/Z drehen.
  Z zeigt nach oben. Die Drehung wird nicht gespeichert.
- **Pro Wolke**: PointSize und Shader-Auswahl:
  - *Default*: Farbe nach Entfernung
  - *Monocolor*: einheitliche Farbe
  - *Cube*: jeder Punkt als Würfel, jede Seite eigene Farbe (+X rot, −X cyan, +Y grün, −Y magenta, +Z blau, −Z gelb); Kantenlänge = PointSize in cm
  - *Sphere*: jeder Punkt als Kugel mit denselben Richtungsfarben, stufenlos; Durchmesser = PointSize in cm
  - *ColorGradient*, *Debug*: Farbverläufe nach Entfernung
  - *zMap*: Farbe nach Höhe (Z)
  - *RGB*: Farben aus der Datei (falls vorhanden; solche Wolken starten automatisch damit), runde Punkte, mit „Use shadow“ kugelartig schattiert
  - *RGB Sphere*: echte Kugeln wie *Sphere*, aber in den Farben aus der Datei; „Use shadow“ schattiert mit Licht von der Kamera
- **Replay / Wegpunkt-Tabelle**: aufgenommene Wegpunkte abfahren, optional als Schleife und als
  Video aufzeichnen (MJPG-`.avi` mit 25 Bildern/s im Ordner `records/` des Projekts). Die Fahrt
  hat live dasselbe Tempo wie im Video (2,5 m/s bzw. 50°/s), unabhängig von der Bildrate;
  Wegpunkte auswählen, löschen, verschieben.
