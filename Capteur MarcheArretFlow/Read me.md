#Capteur de niveau et capteur Marche/Arrêt#

Première version du firmware des capteurs de niveau (US-100) et capteur Marche/Arrêt pour
les pompes.

*Fonctions:*

- Mesure de distance avec le senseur de distance US-100
- Détection marche/arrêt pour 1 moteur. Entré avec isolation optique.
- Input d'état ouvert ou fermé pour 4 valves (A, B, C, D)
- Sortie pour un Led RGB externe reproduisant l'état du led RGB du Photon
- Sortie pour un led externe indiquant la prise de mesure.
- Tampon interne de 300 événements en cas de perte de réseau.
- Mesure 1 fois/seconde quand l'activité est détecté, réduit à une fois aux 5 secondes en cas d'inactivité.
- Publication à interval fixe de X minutes en cas d'inactivité. Programmabe à distance.
- Sérialisation des données
- Formattage JSON des données (exemple):
	event: Distance
	data: {"data":"{'*noSerie*': 126,'*eTime*': 4564134,'*eData*': 253}","ttl":"60","published_at":"2015-10-23T16:55:46.620Z","coreid":"42002a000847343339373536"}

*À faire:*

- Faire une version avec les fonctionnalités de base sans code de capteur
- Mesure de la température interne
- Contrôle du circuit de chauffage
- Mesure de la température externe si le capteur utilisé n'est pas le US-100
- Routine de lecture de distance pour un capteur robuste
