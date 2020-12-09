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


03.12.2020 - Es geht weiter mit dem ToolsLocker!

Erste Versuche waren bereits erfolgreich und Michael M. hat bereits den ersten Platinenentwurf für jeweils 4 Türen fertig gestellt.

Das Konzept sieht wie folgt aus:
- RFID-Leseeinheit mit Display - wie auch schon bei den anderen Maschinen
- 3 Interface Module die mit dem Controller über I2C-Bus kommunizieren
- jedes Interface Module kann 4 elektrische Schlösser und 4 Drucktaster mit LED-Indikator bedienen
- es gibt 2 Netzteile: 1 kleines für die Elektronik und ein Leistungsstarkes 12V Netzteil für die Türschlossmagneten
- der Prozessor kommuniziert über XBee ZBee mit unserem SYMCON-System, welches alle Überprüfungen machen wird, Freigaben erteilt und alle Vorgänge in einer Datenbank loggt

Hier die [System Blockschaltbilder](doc/ToolsLock_Schaltung_B0.pdf)<br>

Wen es interesseirt findet hier [das Schaltbild](doc//ToolsLockTreiberI2CML_SCH.PDF) der Interface Platine<br>

Hier schon mal eine Idee, wie die PCBs für den ToolsLocker aussehen werden:

![Bildschirmfoto 2020-12-05 um 17 13 34](https://user-images.githubusercontent.com/42463588/101286534-4fc3b400-37eb-11eb-9eb7-2b88002af4fe.png)

Grober Funktionsablauf (Vorschlag):

