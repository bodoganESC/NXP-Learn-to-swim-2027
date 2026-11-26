Inainte de a folosi informatiile din acest document, va rog sa cititi o data tot mesajul.

UPDATE: echivalentul pentru majoritatea pasilor de mai jos (copiere fisiere in container si compilare) s-au incercat a fi inclusi in scriptul patch.sh, dar acesta nu a fost verificat inca... Ar trebui sa se ocupe de initializare chei ssh locale, autorizarea cheii in container si apoi de restul pasilor.

Directorul contine fisiere care trebuie adaugate/inlocuite in containerul simulatorului nxpcup23v1 pentru a obtine o forma alfa a simulatorului de camera pixy2 pentru detectia de linii pe care l-am construit pe baza codului pe care l-am extras din sursele firmware-ului camerei (fisierele .7z)
Au fost extrase din masina de docker si arhivat folosind comenzile urmatoare (va vor indica unde sa amplasati pachetele in repo de ros):
git clone --depth=1 ssh://dock:/home/t/ros2ws/src/pixy2_interfaces
7z a pixy2_interfaces.7z pixy2_interfaces
git clone --depth=1 ssh://dock:/home/t/ros2ws/src/pixy_sim
7z a pixy_sim.7z pixy_sim
git clone --depth=1 ssh://dock:/home/t/ros2ws/src/nxp_cup_vision
7z a nxp_cup_vision.7z nxp_cup_vision

Pentru copiere in docker (dupa ce porniti containerul cu scriptul resume.sh), dintr-un terminal local (din afara containerului) pentru fiecare arhiva 7z se ruleaza comenzile urmatoare (adaptate potrivit):
7z x nxp_cup_vision.7z # extragere arhiva
rm -rf nxp_cup_vision/.git # pentru a nu corupe istoricul git din container 
scp -r nxp_cup_vision dock:/home/t/ros2ws/src/nxp_cup_vision
rm -rf nxp_cup_vision


Dupa mutarea fisierelor va trebui sa le compilati cu urmatoarele comenzi rulate din directorul ~/ros2ws (din container):
. ~/.bashrc.sh # pentru a pregati workspaceul ros2
colcon build --packages-select pixy2_interfaces # cea mai simpla
colcon build --packages-select pixy_sim # cea mai simpla

Eventual puteti alege un mod specific de compilare pentru pixy_sim ca in exemplele urmatoare:
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select pixy_sim
colon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-select pixy_sim
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select pixy_sim

Dupa ar trebui sa poata fi rulat, dar retineti ca toate optiunile sunt momentan 'hardcoded' (consultati fisierul pixy.cpp pentru esenta (topicuri) si fw.cpp, fw.hpp pentru detaliile eventual avansate legate de parametri de detectie a liniilor).
Rulare:
. ~/.bashrc.sh # pentru a pregati workspaceul ros2 sa gaseasca modulul nou
ros2 run pixy_sim pixy

Nodul implicit nu afiseaza nimic, dar face subscribe pe topicul de imagini capturate de la camera din simulator, ruleaza algoritmul de la camera (cand apare frame nou) si apoi publica imaginea monocrom redusa si informatii despre linii si intersectii.
In nodul aim_line_follow probabil veti vrea sa faceti subscribe la topicurile din acest noul nod folosind topicurile noi (trebuie adaugate la dependinte modulele pixy2_interfaces si pixy_sim pentru a putea fi folosite mesajele, vedeti fisierele package.xml si CMakeLists.txt din aim_line_follow). Sunt convins ca va trebui sa fiti creativi in partea de integrare, dar AI-urile e posibil sa va raspunda la unele intrebari...

Atunci cand simulatorul (Ignition/Gazebo) ruleaza si face capturi puteti observa streamingul cu comenzi de tipul
ros2 topic echo lines

Suplimentar atasez si cateva fisiere din simulator pe care stiu sigur ca le-am modificat pentru a reproduce conditii similare traseului din laborator (posibil sa trebuiasca si altele, dar contez pe intuitia voastra sa corectati eventualele lipsuri).
Le-am extras din masina de docker folosind comenzile de mai jos (va vor ajuta sa stiti unde sa inlocuiti fisierele).

scp dock:~/ros2ws/nxp_fuel/models/Raceway3/model.sdf ./modelt.sdf    # model masina reparat si completat cu o reprezentare simplista a camerei, adjustati pozitionare/orientarea dupa preferinta... fov e reglat aproape de realitate
scp dock:~/ros2ws/nxp_fuel/models/Raceway3/model.config ./modelt.config    # model masina reparat si completat cu o reprezentare simplista a camerei, adjustati pozitionare/orientarea dupa preferinta... fov e reglat aproape de realitate
scp dock:~/ros2ws/nxp_fuel/models/Raceway3/meshes/Track.dae ./Track.dae #traseul printat din laborator, momentan fara simbolul de start, daca il deschideti cu blender il puteti edita...)
scp dock:~/ros2ws/src/sim_ignition_bringup/config/gen_params.json ./gen_params.json #reconfigurare lansare noduri... pozitie si orientare masina + altele...
scp dock:~/ros2ws/nxp_fuel/models/Buggy3/model.sdf ./model.sdf    # model masina reparat si completat cu o reprezentare simplista a camerei, adjustati pozitionare/orientarea dupa preferinta... fov e reglat aproape de realitate
scp dock:~/ros2ws/nxp_fuel/worlds/gen.world.jinja ./gen.world.jinja   # e dezajctivata generarea umbrelor, foarte important altfel se pare ca pe unele masini randeaza defectuos traseul

Pentru a importa fisierele de mai sus rulati comanda scp cu cei doi parametri (sursa, destinatie) inversati: eg: scp ./modelt.sdf dock:~/ros2ws/nxp_fuel/models/Raceway3/model.sdf 
(trebuie creat directorul /home/t/ros2ws/nxp_fuel/models/Raceway3/meshes in container inainte de a copia fisierele)

Acolo unde vreti sa vedeti ce s-a modificat fata de vers originale folositi comanda git diff dupa ce ati suprascris fisierele.

Eu am lucrat cu docker, dar puteti lucra si direct pe masina... poate fi mult mai confortabil...
As recomanda sa folositi visual studio code indiferent de abordare: local merge bine navigarea pe sintaxa, cu docker, folosind pluginul de SSH fs remote nu merge navigarea facila prin cod ci doar vizualizare/editare/highlighting...
Pentru a adauga remote folder in workspace din docker folositi credentialele prezentate de scriptul resume.sh la lansare: host 0.0.0.0 port 2222 user t paola testtest. Eventual setati root-ul direct in /home/t...

Desi lung, acest mesaj este cu siguranta incomplet... dar reprezinta un ghid orientativ
