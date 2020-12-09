# ToolsLocker
Schrank mit 10 Türen für empfindliche/wertvolle Werkzeuge - Entnahme per Mitglieder RFID-Chip<br>
Geloggt wird UID2, TimeStamp Entnahme, Fachnummer(1 - 10), TimeStamp Rückgabe
<br><br>
Brain Storming:
- Schrank mit 10 Fächern. Geplant ist 2 Fächer mit doppelter Höhe einzurichten
  
![Locker_2](images/locker_2.png)
![Locker_3](doc/Schrank.jpg)

- Verriegelung mechanisch (mit Hand zudrücken)
- Entriegelung elektromagnetisch (12V DC 2A Puls)

![Lock_1](images/Lock_1.png)

![Abmessungen](doc/Abmessungen.png)

- single RFID-Lesegerät mit Display
- jede Tür hat einen Taster (zum öffnen) und eine LED
- XBee/ZBee Modul Funkbrücke zu SYMCON

<br>
<h1>Realisierungsvorschläge</h1> <br>

[Zum Projektschaltbild](doc/ToolsLock_Schaltung_B0.pdf)<br>
[Zum Schaltbild](doc//ToolsLockTreiberI2CML_SCH.PDF)<br>

03.12.2020 - Es geht weiter mit dem ToolsLocker!

Erste Versuche waren bereits erfolgreich und Michael M. hat bereits den ersten Platinenentwurf für jeweils 4 Türen fertig gestellt.

Hier schon al eine Idee, wie die PCBs für den ToolsLocker aussehen werden:

![Bildschirmfoto 2020-12-05 um 17 13 34](https://user-images.githubusercontent.com/42463588/101286534-4fc3b400-37eb-11eb-9eb7-2b88002af4fe.png)

Grober Funktionsablauf (Vorschlag):

