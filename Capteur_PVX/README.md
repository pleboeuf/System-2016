#Capteur de niveau et capteur Marche/Arrêt#

Première version du firmware des capteurs "Pump On/Off" et Vacuum pour
les relâcheurs.

*Fonctions:*

- Mesure de vide dans le relâcheur
- Détection marche/arrêt pour 1 moteur. Entré avec isolation optique.
- Sortie pour un Led RGB externe reproduisant l'état du led RGB du Photon
- Sortie pour un led externe indiquant la prise de mesure.
- Tampon interne de 250 événements en cas de perte de réseau.
- Mesure 1 fois/seconde quand l'activité est détecté
- Publication à interval fixe de 30 minutes en cas d'inactivité. Re-programmabe à distance.
- Sérialisation des données
- Formattage JSON des données (exemple):
	event: Distance
	data: {"noSerie": 3852,"generation": 1453566492,"timestamp": 1453758990,"timer": 4439195,"eData":22,"eName": "sensor/sensorTemp","replay":0}

*À faire:*
