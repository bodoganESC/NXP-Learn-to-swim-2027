# 🏎️ Learn to Swim — NXP Cup EMEA 2026/2027

Repo-ul oficial al echipei **Learn to Swim** pentru **NXP Cup EMEA 2026/2027** (competiția: **17 aprilie 2027**).

Momentan suntem în faza de simulare (`sim24`): testăm detecția de traseu (segmente Pixy2 + transformare ortogonală) și algoritmii de control în Python înainte să primim kit-ul fizic (cel mai probabil **MR-Buggy3** / platformă NXP echivalentă) și să portăm totul în Embedded C.

---

## 👥 Echipa & Împărțirea Rolurilor

* **Bogdan** — *Arhitectură, Calibrare Cameră & Percepție (`perception.py`)*
  * Transformarea segmentelor Pixy2 din pixeli în coordonate reale în metri (`cam_coord_to_orto_m`), clasificarea marginilor stânga/dreapta (inclusiv fallback pe o singură linie în curbele strânse `R170`) și detecția intersecțiilor / liniei de start.
* **Octav** — *Control, Dinamică & Clasa Driver (`controller.py`, `DriverNxp2027`)*
  * Implementarea clasei proprii derivate din `Driver`, regulatorul PID de direcție, profilul adaptiv de viteză (accelerație pe linii drepte, frânare înainte de curbe) și oprirea la max. `10 cm` în fața cubului alb de final.
* **Andrei** — *Particularizare Mecanică, Piste de Test & Telemetrie (`telemetry.py`, `carsim.py`, `VirtualCar`)*
  * Configurarea parametrilor fizici ai mașinii și camerei în simulator, construirea pistelor de test (șicane, intersecții 45°/90°), logarea timpilor pe tur și organizarea testelor comparative.

---

## 📁 Structura Proiectului

```text
learn-to-swim-nxp/
├── .gitignore
├── README.md
├── docs/                  # Regulament 2026/2027, dimensiuni pistă, scheme electrice, notițe
├── sim24/                 # Simulatorul 3D Python + modulul C++ pixy2sim
│   ├── pixy2sim/          # Codul sursă C++ al camerei simulate (pixy.cpp, fw.cpp, setup.py)
│   ├── carsim.py          # Randare OpenGL & buclele de simulare
│   ├── model.py           # Fizica mașinii, camera & clasa Driver
│   ├── perception.py      # [Bogdan] Procesare segmente Pixy2 -> coordonate ortogonale
│   ├── controller.py      # [Octav] PID direcție & control viteză
│   ├── telemetry.py       # [Andrei] Timpi pe tur, log-uri & debug overlay
│   ├── pixy0.prm          # Parametrii standard Pixy2
│   ├── requirements.txt
│   ├── setup.sh
│   └── run.sh
├── firmware/              # [Din Ianuarie] Portarea algoritmului în Embedded C/C++
└── hardware/              # [Opțional] Modele 3D / laser cut, BOM, schițe cablare