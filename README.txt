In resources/initPose.txt si può modificare il tempo per raggiungere quella posizione


-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*

I riferimenti fatti di seguito si riferiscono ai file che si trovano nella cartella uca_franka.
Ci sono da modificare i path assoluti di alcuni file.

In CMakeLists.txt
Righe 34, 72

In include/utils_cam_uca.hpp
Righe 202, 237, 240, 242

In detection_aspirazione.cpp
Righe 35, 46, 48

In detection_scarico.cpp
Righe 35, 46, 48

In graspAspirazione.cpp
Righe 55, 69, 75, 163

In graspScarico.cpp
Righe 53, 67, 73, 157

In moveCobot.cpp
Righe 52, 68, 74, 169, 171

In openGripper.cpp
Righe 16, 28

-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*

Per effettuare la compilazione, dopo aver fatto tutte le modifiche di cui sopra, da terminale andare nella cartella 
libfranka/build 
e lanciare il comando
make

-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*

Per testare occorre lanciare i SOLI codici che fanno i calcoli ma fanno stare fermo il robot, cioè:
dalla cartella build/uca_franka lanciare prima
./detection_scarico <robot_ip>

e poi 
./graspScarico <robot_ip> <path_to shatf.txt file>

Quando si lancia il detector la camera deve essere collegata.
Se questi due funzionano non mi aspetto particolari problemi quando andremo a eseguire il caso d'uso nel complesso