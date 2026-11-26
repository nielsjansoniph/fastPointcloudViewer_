# CLAUDE.md – Einarbeitung für Agenten

Fast Point Cloud Viewer (`fpcv`): C++17/OpenGL-3.3-Viewer für große Punktwolken (.ply/.pcd/.obj)
mit First-Person-Navigation, mehreren Darstellungs-Shadern und Kamerafahrten mit Videoaufnahme.
Bedienung und Nutzersicht: siehe `README.md`.

## Bauen, starten, testen (Linux, Ubuntu 22.04)

```bash
cmake -S . -B build/linux -DCMAKE_BUILD_TYPE=Release   # einmalig bzw. nach neuen Quelldateien
cmake --build build/linux -j
./build/linux/fpcv [wolke.ply|wolke.pcd|modell.obj ...] # aus beliebigem Ordner startbar
```

- Shader werden zur Laufzeit aus `<Projekt>/shaders/` geladen (Pfad per CMake als `FPCV_SOURCE_DIR`
  einkompiliert, siehe `src/Paths.h`; überschreibbar mit Umgebungsvariable `FPCV_SHADER_DIR`).
  Shader-Änderungen brauchen keinen Rebuild, nur einen Neustart. Videos landen in `<Projekt>/records/`.
- Neue `.cpp`-Dateien in `CMakeLists.txt` (`add_executable`) eintragen.
- Testdaten (exFAT-Laufwerk, nur lesen): `/mnt/5AED-2AE3/Scans/Halle1_V3.obj` – 938 000 Punkte mit
  Farben, lädt in ~1 s, Standard-Test. Dort auch große Wolken ohne Farben, z. B.
  `scans_hmv2_clean.ply` (102 Mio. Punkte, 2,9 GB). `jurgensen_merged.ply` (6,8 Mio.) liegt im
  OneDrive-Ordner `Punktwolken`, der von Linux aus derzeit nicht erreichbar ist.
- Es gibt keine automatisierten Tests. Prüfen = bauen, starten, Konsole auf
  `SHADER_COMPILATION_ERROR` prüfen, Screenshot ansehen:
  ```bash
  (./build/linux/fpcv /mnt/5AED-2AE3/Scans/Halle1_V3.obj > log.txt 2>&1 &); sleep 7
  WID=$(xwininfo -root -tree | grep "Fast first person" | awk '{print $1}' | head -1)
  import -window "$WID" shot.png; pkill -x fpcv          # NICHT pkill -f (trifft die eigene Shell)
  ```
  `xdotool` ist nicht installiert – Klicks/Tasten lassen sich nicht simulieren. Für Tests eines
  Zustands (Shader, Kamerafahrt) vorübergehend Code einfügen (z. B. per `getenv`-Schalter in
  `Viewer::loadCloud` oder `CameraPath::update`) und danach wieder entfernen. Videos prüfen mit
  `ffprobe -count_frames`, Einzelbilder mit `ffmpeg -vf select=...`.

## Abhängigkeiten

- **Linux**: PCL 1.12, OpenCV 4.5, glm aus apt. GLFW 3.4, glad 0.1.36, Dear ImGui 1.91.5 und
  nativefiledialog-extended 1.2.1 (Portal-Backend, kein GTK) holt CMake per `FetchContent`.
- **Windows**: vcpkg (pcl, opencv, glad, glfw3, imgui, nfd), `if(WIN32)`-Zweig in `CMakeLists.txt`.
  Beide Zweige erhalten; Code und Shader müssen mit MSVC *und* GCC/Mesa funktionieren.
  Der Windows-Build wird hier nicht getestet.

## Architektur (`src/`)

| Datei | Inhalt |
|---|---|
| `main.cpp` | Kommandozeile, GLFW/glad/ImGui-Init, Hauptschleife (`viewer.frame()`, `afterRender()`, Swap) |
| `Viewer.*` | Anwendung: besitzt Kamera, Wolken, Shader-Tabelle, Achsen, Kamerafahrt, Modellrotation; Menü, Eingabe, Rendering |
| `PointShaders.*` | **Tabelle aller Punkt-Shader**: Name, Shader-Dateien, Menü-Optionen, Uniforms. Neuer Shader = ein Eintrag in `createPointShaders()` |
| `CameraPath.*` | Wegpunkte, Interpolation (0,1 m bzw. 2° pro Schritt), Replay, Videoaufnahme (OpenCV, MJPG). Aufnahme: 1 Schritt = 1 Videobild (25 Bilder/s); live: `replayPos += DeltaTime × 25`, Pose zwischen zwei Schritten interpoliert → gleiches Tempo wie im Video |
| `Axes.*` | Koordinatenkreuz + Beschriftung (ImGui-ForegroundDrawList) |
| `Camera.*` | Position/Orientierung, **Z ist oben**, WASD/RF in m/s × Frame-Zeit, Maus-/Pfeiltastenrotation, setzt gemeinsame Uniforms (`Camera::Matrix`) |
| `Cloud.*` | .obj: eigener Leser (`readObj`, nur `v x y z [r g b]`-Zeilen, Farben 0..1 oder 0..255, locale-unabhängig per `from_chars`, dreht Y-oben → Z-oben: (x, y, z) → (x, −z, y)). Lädt .ply/.pcd über PCL (wirft `std::runtime_error`): einmal als `PCLPointCloud2` lesen, bei Feld `rgb`/`rgba` → `PointXYZRGBA`, sonst `PointXYZ`. GPU: Positionen 12 Byte/Punkt (Attribut 0), Farben 4 Byte/Punkt (Attribut 1, nur bei `hasColor`), keine Punktdaten im RAM; Bounding Box, `shaderType` (Index in die Shader-Tabelle), `pointssize`, `point_color` |
| `Shader.*` | Shader-Programm; löst `#include "datei.glsl"` in Shader-Dateien auf |
| `VAO.*`, `VBO.*` | Dünne OpenGL-Wrapper |
| `Paths.h`, `UiWidgets.h` | Laufzeitpfade; `SliderFloatWithButtons` (Regler mit −/+) |

### Ablauf eines Frames (`Viewer::frame`, Reihenfolge ist wichtig)
1. ImGui-Frame, Menü (`drawUi`), ggf. Datei-Dialog
2. Fenstergröße → Kamera; Eingabe (nur wenn keine Kamerafahrt läuft und die Maus nicht über dem Menü ist)
3. `CameraPath::update` (startet Fahrt, setzt Kamera-Pose), `camera.updateMatrix`
4. `render`: Modellmatrix, Wolken (Uniforms aus der Shader-Tabelle), Achsen, `ImGui::Render()`
   erst hier (Achsenbeschriftung braucht die aktuelle Kameramatrix), Menü nur ohne Kamerafahrt
5. `afterRender` → `CameraPath::afterRender` liest das Bild **vor** `glfwSwapBuffers` für das Video aus

### Shader (`shaders/`)
| Index | Name | Dateien | Hinweis |
|---|---|---|---|
| 0 | Default | `points.vert` + `default.frag` | Farbe nach Tiefe (HSV) |
| 1 | Monocolor | `points.vert` + `monoColor.frag` | Standard beim Laden (`kDefaultPointShader`) |
| 2 | Cube | `cube.vert` + `cube.frag` | Raycast achsenparalleler Würfel |
| 3 | ColorGradient | `points.vert` + `colorgradient.frag` | Farbton nach Entfernung; Helligkeit von Start Brightness (nah) bis End Brightness (fern) |
| 4 | Debug | `points.vert` + `debug.frag` | geteilte Ansicht (`split`) |
| 5 | zMap | `points.vert` + `zmap.frag` | Farbe nach Höhe **z** |
| 6 | Sphere | `sphere.vert` + `sphere.frag` | Raycast Kugel |
| 7 | RGB | `rgb.vert` + `rgb.frag` | Farben aus der Datei, Standard für Wolken mit Farben (`kColorPointShader`) |
| 8 | RGB Sphere | `sphere.vert` + `rgbsphere.frag` | Raycast Kugel in Dateifarben, Licht von der Kamera bei „Use shadow“ |
| – | Achsen | `axis.vert` + `axis.frag` | Koordinatenkreuz |

- `common.glsl`: gemeinsame Uniforms (`near`, `far`, `useDepthOn…`, `useShadow`, `cameraPosition`)
  und Funktionen (`linearizeDepth`, `logisticDepth`, `hsv2rgb`, `map`, `spriteHeight`). Diese Uniforms
  daher **nicht** nochmal im Fragment-Shader deklarieren.
- `points.vert` hat `uniform mat4 model` (Modellrotation); `pointPosition` ist in Weltkoordinaten.
- Cube/Sphere/RGB Sphere (`programPointSize = true`): Vertex-Shader setzt `gl_PointSize` aus Weltgröße
  (= PointSize in cm) und Entfernung; Fragment-Shader schneidet den Sichtstrahl analytisch
  (`invCamMatrix`, `viewportSize`) und schreibt `gl_FragDepth`. Der Kugelschnitt steckt in
  `sphere.glsl` (`raycastSphere`), gemeinsam für Sphere und RGB Sphere.
- Projektion fest `45°, 0.1, 500` (`Viewer.cpp`); die Regler Near/Far steuern **nur** die
  Helligkeitsabnahme in den Fragment-Shadern.

## Stolperfallen (alle real aufgetreten)

- **GLSL für Mesa**: keine Default-Werte für Funktionsparameter – NVIDIA/Windows akzeptiert das,
  Mesa nicht. Uniform-Initialisierer (`uniform float x = 1.0;`) sind ok.
- **ImGui-Tasten**: immer `ImGuiKey_W` usw., niemals `(ImGuiKey)GLFW_KEY_W` – wird stillschweigend ignoriert.
- **GCC**: Member mit gleichem Namen wie sein Typ (`VAO VAO;`) muss als `::VAO VAO;` geschrieben werden.
- `<glad/glad.h>` immer vor `<GLFW/glfw3.h>` einbinden; `imgui_impl_opengl3_loader.h` nie einbinden.
- Bewegungen/Rotationen mit `ImGui::GetIO().DeltaTime` skalieren – das Programm läuft mit ~120–240 FPS.

## Git / Dateisystem

- Linux-Arbeitskopie: `/home/niels/fastPointcloudViewer_` (ext4, eigener Clone, nur LF).
  Die Windows-Arbeitskopie ist ein separater Clone; Abgleich nur über GitHub (Push/Pull).
  Das Repo speichert LF (`.gitattributes`: `* text=auto`), unter Windows ggf. CRLF in der Arbeitskopie.
- Keine globale Git-Identität unter Linux: für Commits
  `git -c user.name="$(git log -1 --format=%an)" -c user.email="$(git log -1 --format=%ae)" commit …`.
- Binärdateien sind nicht im Repo (seit 2026-11-26): `build/`, `*.exe`, `*.dll`, `*.pdb` sind ignoriert.
  Unter Windows kopiert der vcpkg-Build die DLLs selbst nach `build/Debug`.
- Push zu GitHub bricht gelegentlich mit Timeout ab – einfach erneut versuchen.
- Große Testdaten (`*.ply`, `*.pcd`, `records/`) sind ignoriert, nicht committen. Keine
  Wildcard-Löschungen in `records/` (könnte Videos des Nutzers treffen), nur eigene Dateien gezielt.

## Offene Punkte (Stand 2026-11-26)

Mit dem Nutzer besprochen, aber noch nicht entschieden bzw. umgesetzt:
- **Windows-Build prüfen**: octomap/assimp wurden auch im `WIN32`-Zweig entfernt, Umstrukturierung
  nach `src/`/`shaders/` – unter Windows noch nicht gebaut.
- **LOD/Culling** für sehr große Wolken (Blöcke + Frustum-Culling, gröbere Darstellung in der Ferne):
  größerer Umbau, nur bei Bedarf.
- **Videoaufnahme**: früher ~3 Bilder/s auf dem NTFS/OneDrive-Laufwerk. Gemessen 2026-11-26 auf ext4
  (1600×900, Halle1_V3.obj): glReadPixels 5,3 ms, flip 0,4 ms, MJPG-Schreiben 6,4 ms, ~80 Bilder/s.
  Ursache war also vermutlich das Laufwerk (oder ntfs-3g). Unter Windows nicht gemessen.

## Arbeitsweise mit dem Nutzer

- Kommunikation auf Deutsch. Code-Kommentare und Commit-Messages auf Englisch.
- Der Nutzer entscheidet über Commit und Push; häufig „committen, später pushen“.
- Nicht breit über `/home` oder `/media` nach Dateien suchen – nach dem Pfad fragen.
