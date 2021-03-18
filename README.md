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

03.12.2020 - Es geht weiter mit dem ToolsLocker!

Erste Versuche waren bereits erfolgreich und Michael M. hat bereits den ersten Platinenentwurf für jeweils 4 Türen fertig gestellt.

Das Konzept sieht wie folgt aus:
- RFID-Leseeinheit mit Display - wie auch schon bei den anderen Maschinen
- 3 Interface Module die mit dem Controller über I2C-Bus kommunizieren
- jedes Interface Module kann 4 elektrische Schlösser und 4 Drucktaster mit LED-Indikator bedienen
- es gibt 2 Netzteile: 1 kleines für die Elektronik und ein Leistungsstarkes 12V Netzteil für die Türschlossmagneten
- der Prozessor kommuniziert über eine XBee/ZBee Modul Funkbrücke mit unserem SYMCON-System, welches alle Überprüfungen machen wird, Freigaben erteilt und alle Vorgänge in einer Datenbank loggt

Hier dazu die [System Blockschaltbilder](doc/ToolsLock_Schaltung_B0.pdf)<br>

Wen es interessiert findet hier [das Schaltbild](doc//ToolsLockTreiberI2CML_SCH.PDF) der Interface Platinen<br>

So wird das PCBs für den ToolsLocker aussehen:

![Bildschirmfoto 2020-12-05 um 17 13 34](https://user-images.githubusercontent.com/42463588/101286534-4fc3b400-37eb-11eb-9eb7-2b88002af4fe.png)

Status - heute Mittwoch der 09.12.2020:

- Laborversuchsaufbau mit einem Schloss und einem Taster wurde bereits aufgebaut und erfolgreich getestet.
- Layout PCB Interface Platine ist abgeschlossen - 5 Platinen sind bestellt und bereits auf dem Weg aus China.


Nächste Schritte:<br>
Es ist nun die Zeit gekommen, wo wir Unterstützung von 1 bis 2 zuverlässigen Mechanikern gebrauchen könnten. Es geht darum, zunächst für eine Tür das ausgewählte Schloss und den LED-Taster zu installieren, um am eigentlichen Objekt die Machbarkeit zu demonstrieren und mit der Verwaltungssoftware beginnen zu können.

Erst nach positiver Beurteilung würden wir die restlichen Teile bestellen, um den Aufbau voran zu treiben.

Also - Freiwillige mögen sich bei uns melden.

Grober Funktionsablauf (Vorschlag):
- der ToolsLocker ist an der Netzspannung angeschlossen und mit dem SYMCON System synchronisiert
- zum 'einloggen' darf das Mitglied an keiner anderen Maschine 'eingeloggt' sein
- RFID-Chip an Lesegerät halten
- System prüft ob Mitglied/RFID-Chip registriert ist und bestätigt entsprechen via Display
- 

<h2>16.12.2020 - Post aus China</h2>

![IMG_7514](https://user-images.githubusercontent.com/42463588/102316586-81204a80-3f76-11eb-94c7-06950fffdf84.jpg)

![IMG_7515](https://user-images.githubusercontent.com/42463588/102316639-9ac19200-3f76-11eb-9788-50943ebae297.jpg)

<h2>Tolo-Farbkodierung Kabel / Stecker </h2>

<h4>Elektromagnetisches Schloss:</h4>

 - Anschluss Schloss (2-poloig male)
 - Anschluss Schlosskontakt (2-polig female)


<h4>LED-Taster-Anschluss (3-polig female):</h4>

- Grau ist +5V
- Lila ist LED (Kathode)
- Blau ist Tasterkontakt

<h4>Signalkabel (grau):</h4>

- Schwarz & rot sind für das Schloss (2-polig female)
- Weiß und Gelb/Grün sind für den Schloßkontakt (2-polig male)
- Grau (+5V), braun (LED) und Blau (Kontakt) sind für den Taster (3-polig male)
<h2>Final Controller Box</h2>

![IMG_7516](https://user-images.githubusercontent.com/42463588/111650629-c36f5700-8805-11eb-865c-ec82ad748833.png)
