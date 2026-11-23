# CLAUDE.md – Einarbeitung für Agenten

Fast Point Cloud Viewer (`fpcv`): C++17/OpenGL-3.3-Viewer für große Punktwolken (.ply/.pcd)
mit First-Person-Navigation, mehreren Darstellungs-Shadern und Kamerafahrten mit Videoaufnahme.
Bedienung und Nutzersicht: siehe `README.md`.

## Bauen, starten, testen (Linux, Ubuntu 22.04)

```bash
cmake -S . -B build/linux -DCMAKE_BUILD_TYPE=Release   # einmalig
cmake --build build/linux -j
cd build/linux && ./fpcv [wolke.ply|wolke.pcd ...]      # MUSS aus build/linux gestartet werden
```

- Shader werden zur Laufzeit relativ über `../../<name>.vert/.frag` geladen → Arbeitsverzeichnis
  muss zwei Ebenen unter dem Projektordner liegen (`build/linux`, unter Windows `build/Debug`).
  Shader-Änderungen brauchen keinen Rebuild, nur einen Neustart.
- Es gibt keine automatisierten Tests. Prüfen = bauen, starten, Konsole auf
  `SHADER_COMPILATION_ERROR` prüfen, Screenshot ansehen:
  ```bash
  (./fpcv /pfad/wolke.ply > log.txt 2>&1 &); sleep 5
  WID=$(xwininfo -root -tree | grep "Fast first person" | awk '{print $1}' | head -1)
  import -window "$WID" shot.png; pkill -f "^./fpcv"
  ```
  `xdotool` ist nicht installiert – Klicks/Tasten lassen sich nicht simulieren. Für Tests eines
  bestimmten Zustands (Shader, Kameraposition) vorübergehend Code nach dem Laden der
  Startdateien in `main.cpp` einfügen und danach wieder entfernen.
- Kleine Testwolke: ASCII-PLY mit `element vertex N` / `property float x|y|z` selbst erzeugen.

## Abhängigkeiten

- **Linux**: PCL 1.12, OpenCV 4.5, assimp, glm aus apt (installiert). GLFW 3.4, glad 0.1.36,
  Dear ImGui 1.91.5 und nativefiledialog-extended 1.2.1 (Portal-Backend, kein GTK) holt CMake per
  `FetchContent` – kein `sudo` nötig. Octomap wird unter Linux nicht gelinkt (im Code unbenutzt).
- **Windows**: vcpkg, `find_package(...)` im `if(WIN32)`-Zweig von `CMakeLists.txt`. Beide Zweige
  erhalten; Änderungen an Code/Shadern müssen mit MSVC *und* GCC/Mesa funktionieren.

## Architektur

| Datei | Inhalt |
|---|---|
| `main.cpp` | Fast alles: Init, Shader-Erzeugung, ImGui-Menü, Hauptschleife, Rendering, Kamerafahrten/Videoaufnahme (OpenCV) |
| `Camera.*` | Position/Orientierung, **Z ist oben** (`Up = (0,0,1)`), WASD/RF-Bewegung (m/s × Frame-Zeit), Mausrotation, `rotate()` für Pfeiltasten, setzt gemeinsame Uniforms (`camMatrix`, `near`, `far`, `cameraPosition`, …) |
| `Cloud.*` | Lädt .ply/.pcd über PCL (wirft `std::runtime_error` bei Fehlern), lädt nur Positionen (12 Byte/Punkt) auf die GPU und behält keine Punktdaten im RAM; hält `numPoints`, Bounding Box (`min`/`max`), VAO, `shaderType`, `pointssize` |
| `shaderClass.*`, `VAO/VBO/EBO.*` | Dünne OpenGL-Wrapper |
| `Mesh.*`, `Texture.*`, `mesh*`, `light*`, `meshTexShader*` | Mesh-Rendering, derzeit auskommentiert/ungenutzt |
| `happly.h`, `stb_image.h` | Eingebundene Header-Bibliotheken – nicht anfassen |

### Hauptschleife in `main.cpp` (Reihenfolge ist wichtig)
1. ImGui-Frame + Menü „Viewer settings“ (Regler, Orientation, pro Wolke Shader-Auswahl, Wegpunkt-Tabelle)
2. Laden über „Load“-Button (`loadCloud`-Lambda, auch für Kommandozeilen-Argumente)
3. Tastatur/Maus → Kamera (`ImGui::IsKeyDown(ImGuiKey_*)`, nur wenn ImGui die Maus nicht will)
4. Kamerafahrt-Logik (Wegpunkte `posPath`/`oriPath`, Interpolation, Aufnahme)
5. `modelMatrix` = Rotation um Mittelpunkt aller Wolken; Wolken zeichnen (Uniforms je `shaderType`)
6. Koordinatenkreuz + Beschriftung (ImGui-ForegroundDrawList)
7. `ImGui::Render()` erst hier (damit Achsenbeschriftung nach dem Kamera-Update entsteht), dann Draw-Data, Swap, ggf. Videoframe

### Shader (`Cloud::shaderType`)
| Typ | Name | Dateien | Hinweis |
|---|---|---|---|
| 0 | Default | `default.vert` + `default.frag` | Farbe nach Tiefe (HSV) |
| 1 | Monocolor | `default.vert` + `monoColor.frag` | Standard beim Laden |
| 2 | RGB sphere | `rgbSphere.frag` | im Menü auskommentiert |
| 3 | Cube | `cube.vert` + `cube.frag` | Raycast achsenparalleler Würfel |
| 4 | ColorGradient | `default.vert` + `colorgradient.frag` | |
| 5 | Debug | `default.vert` + `debug.frag` | geteilte Ansicht (`split`) |
| 6 | zMap | `default.vert` + `zmap.frag` | Farbe nach Höhe **z** |
| 7 | Sphere | `sphere.vert` + `sphere.frag` | Raycast Kugel |
| – | Achsen | `axis.vert` + `axis.frag` | Koordinatenkreuz |

- `default.vert` hat `uniform mat4 model` (Modellrotation); `pointPosition` ist in Weltkoordinaten.
- Cube/Sphere: Vertex-Shader setzt `gl_PointSize` aus Weltgröße und Entfernung
  (`GL_PROGRAM_POINT_SIZE` nur für diese beiden aktivieren); Fragment-Shader schneidet den Sichtstrahl
  analytisch (`invCamMatrix`, `viewportSize`), schreibt `gl_FragDepth`. Größe = `pointssize` in cm.
- Die Projektion ist fest `updateMatrix(45°, 0.1, 500)`; die Regler Near/Far steuern **nur** die
  Helligkeitsabnahme (`linearizeDepth` + `logisticDepth` in den Fragment-Shadern).

## Stolperfallen (alle real aufgetreten)

- **GLSL für Mesa**: keine Default-Werte für Funktionsparameter (`float f(float x = 1.0)`) –
  NVIDIA/Windows akzeptiert das, Mesa nicht. Uniform-Initialisierer (`uniform float x = 1.0;`) sind ok.
- **ImGui-Tasten**: immer `ImGuiKey_W` usw., niemals `(ImGuiKey)GLFW_KEY_W` – neuere ImGui ignoriert das stillschweigend.
- **GCC**: Member mit gleichem Namen wie sein Typ (`VAO VAO;`) muss als `::VAO VAO;` geschrieben werden.
- `imgui_impl_opengl3_loader.h` nicht einbinden (überschreibt glad-Funktionen).
- Bewegungen/Rotationen immer mit `ImGui::GetIO().DeltaTime` skalieren – das Programm läuft mit ~240 FPS.
- Neue Regler mit −/+-Buttons: `SliderFloatWithButtons(label, &wert, min, max, faktor, format, flags)` in `main.cpp`.

## Git / Dateisystem

- Projekt liegt auf einem **NTFS-Laufwerk in OneDrive**, das Windows und Linux gemeinsam nutzen.
  Arbeitskopie hat CRLF, das Repo speichert LF (`.gitattributes`: `* text=auto`). Beim Bearbeiten
  die vorhandenen Zeilenenden der Datei beibehalten. `git diff` kann ganze Dateien als geändert
  anzeigen (liest die CRLF-Arbeitskopie); maßgeblich ist `git diff --numstat` bzw. `git diff -w`.
- Fehler „Unzureichende Berechtigung … .git/objects“: NTFS-Schreibschutz auf Objekt-Ordnern →
  `find .git -type d ! -writable -exec chmod u+w {} +`.
- Keine globale Git-Identität unter Linux: für Commits
  `git -c user.name="$(git log -1 --format=%an)" -c user.email="$(git log -1 --format=%ae)" commit …`.
- `build/Debug/fpcv.exe` (Windows-Build) ist getrackt und lokal oft geändert – nicht mitcommitten,
  außer der Nutzer will es. `*.pdb` ist ignoriert (GitHub-Limit 100 MB).
- Push zu GitHub bricht gelegentlich mit Timeout ab – einfach erneut versuchen.
- Große Testdaten (`*.ply`, `*.pcd`, `records/`) sind ignoriert, nicht committen.

## Arbeitsweise mit dem Nutzer

- Kommunikation auf Deutsch. Code-Kommentare und Commit-Messages auf Englisch.
- Der Nutzer entscheidet über Commit und Push; häufig „committen, später pushen“.
