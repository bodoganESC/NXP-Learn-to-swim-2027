# 📘 Ghid de Contribuție, Stil de Cod și Reguli Repository — Learn to Swim

Acest document stabilește standardele tehnice ale echipei **Learn to Swim** pentru **NXP Cup EMEA 2026/2027**. Respectarea acestor reguli ne asigură că:
1. Nu ne suprascriem munca în Git și avem mereu un branch `main` funcțional.
2. Codul scris acum în **Python (`sim24`)** va putea fi portat în **Embedded C** pe placa NXP fără să îl rescriem de la zero.

---

## 1. Fluxul de Lucru în Git & GitHub

### 1.1. Regula de Aur a Branch-urilor
* **`main` este protejat:** Nu se face niciodată `git push` direct în `main`. Orice modificare intră doar prin **Pull Request (PR)** aprobat.
* **Branch-uri scurte și izolate:** Fiecare task primește propriul branch creat din `main`, cu unul dintre prefixele:
  * `feat/<descriere-scurta>` — funcționalitate nouă (ex: `feat/orto-lane-detection`, `feat/pid-steering`)
  * `fix/<descriere-scurta>` — repararea unui bug (ex: `fix/intersection-turn-bug`)
  * `tune/<descriere-scurta>` — ajustări de parametri PID, viteză sau cameră (ex: `tune/r170-corner-speed`)
  * `track/<descriere-scurta>` — piste noi de test în `carsim.py` (ex: `track/chicane-stress-test`)
  * `docs/<descriere-scurta>` — documentație și comentarii (ex: `docs/update-contributing`)

### 1.2. Atomic Commits & Tag-uri (Conventional Commits)
Un **commit atomic** conține **o singură modificare logică completă** (nu amesteca în același commit modificări de PID cu modificări de pistă sau redenumiri de fișiere). Dacă trebuie să scrii „și” în mesajul de commit, probabil ai nevoie de două commit-uri separate.

Format obligatoriu: `<tag>: <descriere scurtă la imperativ, fără punct la final>`

| Tag | Când se folosește | Exemplu corect |
| :--- | :--- | :--- |
| **`feat`** | Adaugi o funcționalitate sau un modul nou | `feat: adauga estimarea centrului pistei pe o singura margine` |
| **`fix`** | Repari o eroare de logică sau un crash | `fix: previne impartirea la zero cand nu exista vectori` |
| **`tune`** | Modifici doar constante (PID, viteze, praguri Pixy2) | `tune: ajusteaza kp si kd pentru curbele stranse R170` |
| **`refactor`** | Rearanjezi codul fără să îi schimbi comportamentul | `refactor: extrage calculul erorii laterale in perception.py` |
| **`docs`** | Adaugi/modifici documentație sau docstring-uri | `docs: explica sistemul de coordonate cam_coord_to_orto_m` |
| **`chore`** | Configurări de repo, `.gitignore`, scripturi setup | `chore: actualizeaza regulile de ignorare pentru fisiere pdf` |

### 1.3. Reguli pentru Pull Requests (PR) & Merge
1. **Verificare locală:** Înainte să deschizi un PR, rulează `python carsim.py` și asigură-te că simulatorul pornește și rulează fără erori.
2. **Descriere clară:** Orice PR trebuie să conțină 2–3 rânduri cu:
   * **Ce s-a schimbat** (ex: *Am adăugat filtrarea segmentelor după pantă în `perception.py`*).
   * **Cum s-a testat** (ex: *Testat 3 tururi pe `demoM` și `nxp23f`, mașina nu mai iese în decor la intersecție*).
3. **Code Review:** Cel puțin **1 coleg** trebuie să verifice codul (în special portabilitatea spre C și lipsa bug-urilor evidente) și să dea **Approve**.
4. **Metoda de Merge:** Folosim exclusiv **Squash and Merge** pe GitHub (combină toate commit-urile intermediare din branch într-un singur commit curat pe `main` și șterge automat branch-ul vechi).

### 1.4. Comenzi Git Esențiale de Zi cu Zi
* **Actualizarea branch-ului propriu cu ultimele noutăți din `main`** *(folosește `merge`, nu `rebase`, ca să eviți conflictele complicate)*:
  ```bash
  git checkout main && git pull origin main
  git checkout feat/branch-ul-tau
  git merge main
  ```
* **Ai început să scrii cod din greșeală direct pe `main` (înainte de commit):**
  ```bash
  git checkout -b feat/nume-task   # mută automat modificările nesalvate pe noul branch
  ```
* **Vrei să vezi rapid ce ai modificat înainte să dai `git add`:**
  ```bash
  git status -s
  git diff
  ```

---

## 2. Standarde de Cod Python (Pregătit pentru Portare în Embedded C)

În ianuarie vom traduce logica din Python în C/C++ pe microcontrolerul NXP (unde nu avem `numpy`, alocare dinamică nelimitată sau dicționare flexibile). Scrie codul Python ca și cum ar fi **C cu sintaxă de Python**.

### 2.1. Convenții de Denumire
* **Variabile și funcții:** `snake_case` clar și descriptiv, cu unitatea de măsură în nume acolo unde contează.
  * ✅ `left_border_x_m`, `target_speed_mps`, `compute_steering_curvature()`
  * ❌ `x1`, `val`, `calc()`, `temp2`
* **Constante și parametri de tuning:** `UPPER_SNAKE_CASE` grupate toate la începutul fișierului (în C vor deveni `#define` sau `static const`).
  * ✅ `TRACK_WIDTH_M = 0.55`, `HALF_TRACK_WIDTH_M = 0.275`, `KP_STEER = 3.2`
* **Structuri de date (`@dataclass`):** `PascalCase` — reprezintă exact `struct`-urile pe care le vom defini în headerele `.h` din C.

### 2.2. Ce SĂ FOLOSEȘTI vs. Ce SĂ EVIȚI în Logica de Control
Regula este simplă: **separă matematica de simulator**. Fișierele `perception.py` și `controller.py` nu au voie să importe `pygame`, `OpenGL` sau `cv2`.

| Element | ✅ Recomandat (Ușor de portat în C) | ❌ De evitat în `perception.py` / `controller.py` |
| :--- | :--- | :--- |
| **Structuri de date** | `@dataclass` cu tipuri fixe (`float`, `int`, `bool`) | Dicționare imbricate (`dict` în `dict`), tuple-uri anonime cu 8 elemente |
| **Colecții / Vectori** | Liste simple parcurse cu `for` clasic și lungime maximă cunoscută (Pixy2 dă max. ~10–15 segmente) | `numpy` arrays complexe, list comprehensions imbricate pe 3 niveluri, ` pandas` |
| **Alocare memorie** | Actualizarea câmpurilor dintr-un obiect/struct pre-alocat | Crearea de obiecte noi mari la fiecare cadru (`60 Hz` / `1000 Hz`) |
| **Funcții matematice** | `math.sqrt`, `math.atan2`, `math.sin`, `math.cos`, `abs`, `min`, `max` (au echivalent 1-la-1 în `<math.h>`) | Funcții speciale din `scipy` sau `numpy.linalg` |
| **Tipuri explicite** | Type hints peste tot (`def step(err: float, dt: float) -> float:`) | Variabile care își schimbă tipul din `float` în `None` sau `list` |

### 2.3. Exemplu de Cod Python „C-Ready”
```python
from dataclasses import dataclass
from enum import IntEnum

TRACK_WIDTH_M: float = 0.55
HALF_TRACK_WIDTH_M: float = 0.275
MAX_SEGMENTS: int = 16


class RoadState(IntEnum):
    NORMAL = 0
    SINGLE_LINE = 1
    INTERSECTION = 2
    STOP_CUBE = 3


@dataclass
class Segment2D:
    x0_m: float
    y0_m: float
    x1_m: float
    y1_m: float


@dataclass
class PerceptionOutput:
    lateral_error_m: float = 0.0
    heading_error_rad: float = 0.0
    road_state: RoadState = RoadState.NORMAL
    confidence: float = 0.0
```
*(Acest cod se traduce în C în 2 minute într-un `typedef enum` și două `typedef struct`.)*

---

## 3. Reguli pentru Documentația Scrisă

Documentația din repository (`README.md`, `docs/*.md` și comentariile din cod) trebuie să fie utilă pentru un coleg care deschide proiectul peste 2 luni, nu umplutură:

1. **Comentariile explică „DE CE”, nu „CE”:**
   * ❌ `# aduna 0.275 la x_left` (inutil, se vede din cod)
   * ✅ `# Estimam centrul benzii (55 cm / 2 = 0.275 m) cand marginea dreapta iese din cadru in curbele R170`
2. **Fiecare funcție publică are docstring scurt (3–5 rânduri):**
   * Ce face funcția, ce unități de măsură au parametrii (`m`, `rad`, `s`, `px`) și ce returnează.
3. **Orice decizie de calibrare sau parametru magic se notează în `docs/`:**
   * Când schimbați unghiul camerei, înălțimea ei sau o constantă PID care rezolvă un caz limită (ex. șicana `R400`), notați valorile și motivul într-un fișier Markdown din `docs/` (ex: `docs/tuning_log.md`).
4. **Limba:** Păstrăm codul (nume de variabile, funcții, clase, comentarii tehnice) în **Engleză** (pentru compatibilitate cu SDK-ul NXP și codul existent din `sim24`), iar documentația echipei (`README.md`, `docs/*.md`, descrierile de PR) în **Română** (sau Engleză, consecvent per fișier).

---

## 4. Ce Fișiere Intră în Repository și Ce Nu

Git este făcut pentru **cod sursă și text**, nu pentru fișiere binare mari. Un fișier binar de 20 MB modificat de 5 ori umflă repo-ul la 100 MB pentru toată echipa.

### ✅ Ce PUI în GitHub Repository
* **Cod sursă:** `.py`, `.c`, `.cpp`, `.h`, `.hpp`, `.sh`, `CMakeLists.txt`, `Makefile`, `setup.py`.
* **Fișiere de configurare ușoare (text):** `.gitignore`, `requirements.txt`, `pixy0.prm`, fișiere `.json` sau `.yaml` de parametri.
* **Documentație Markdown:** `README.md`, `CONTRIBUTING.md`, notițe tehnice și jurnale de testare în `docs/*.md`.
* **Fișiere CAD / 3D print ușoare (doar când trecem la hardware):** `.stl`, `.step`, `.dxf` (doar versiunile finale, nu zeci de iterații).

### ❌ Ce NU PUI în GitHub Repository (deja blocate în `.gitignore`)
* **Medii virtuale și dependențe instalate:** `venv/`, `.venv/`, `node_modules/`, `__pycache__/`.
* **Fișiere compilate local:** `*.so` (Linux), `*.pyd` / `*.dll` (Windows), `*.o`, `*.elf`, `*.bin`, `build/`. Fiecare membru își compilează local extensia `pixy2sim`.
* **Documente grele și arhive:** `*.pdf` (datasheet-uri, regulamente), `*.docx`, `*.xlsx`, `*.zip`, `*.7z`. Acestea stau în **Google Drive-ul comun al echipei**, iar în `docs/*.md` punem doar link-ul către ele.
* **Capturi de ecran și video-uri din simulator:** `sim24/snap/`, `*.png`, `*.jpg`, `*.mp4`, `*.log`. Dacă vrei să arăți un bug sau un tur record într-un Pull Request, trage imaginea/video-ul direct în caseta de comentariu a PR-ului pe GitHub (GitHub o găzduiește extern fără să o bage în istoricul Git).
* **Setări personale de IDE:** `.idea/` (PyCharm), `.vscode/` (VS Code), `.DS_Store`.