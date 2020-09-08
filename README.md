# ToolsLocker
Schrank mit 10 Türen für empfindliche/wertvolle Werkzeuge - Entnahme per Mitglieder RFID-Chip<br>
Geloggt wird UID2, TimeStamp Entnahme, Fachnummer(1 - 10), TimeStamp Rückgabe
<br><br>
Brain Storming:
- Schrank mit 10 Fächern. Geplant ist 2 Fächer mit doppelter Höhe einzurichten
  
![Locker_1](images/locker_3.png)
![Locker_2](images/locker_2.png)

- Verriegelung mechanisch (mit Hand zudrücken)
- Entriegelung elektromagnetisch (12V DC 2A Puls)

![Lock_1](images/Lock_1.png)

- single RFID-Lesegerät mit Display und 4x4 Tastatur
- XBee/ZBee Modul Funkbrücke zu SYMCON

<br>
<h1>Realisierungsvorschläge</h1> <br>

[Zum Alpha 0 Blockschaltbild](doc/ToolsLockerSchaltung_A0.pdf)<br>
[Zum Alpha 1 Schaltbild](doc/ToolsLockTreiberI2C_SCH.PDF)<br>
Es gibt bereits ein modifiziertes Schaltbild mit reduzierter Bauelementezahl
[Zum Alpha 2 Schaltbild](doc/ToolsLockTreiberI2CLight_SCH.PDF)