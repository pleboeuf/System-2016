#Pour tout les capteurs de l'érablière#

Fonctions de base des capteurs de l'érablière

*Fonctions:*

- Sortie pour un Led RGB externe reproduisant l'état du led RGB du Photon
- Sortie pour un led externe indiquant la prise de mesure.
- Tampon interne de x (300) événements en cas de perte de réseau.
- Sérialisation des données
- Horodatage à la seconde (timestamp) utilisant l'horloge interne (RTC).
- Horodatage de précision en comptant le nombre de millisecondes depuis
	la mise en marche du capteur.
- Formattage JSON des données (exemple):
	event: Distance
	data: {"data":"{'*noSerie*': 126,'*eDate*': 1413034662,'*eTime*': 4564134,
	'*eData*': 253}","ttl":"60","published_at":"2015-10-23T16:55:46.620Z",
	"coreid":"42002a000847343339373536"}

Tous les capteurs devraient:
- Mesuré 1 fois/seconde quand l'activité est détecté, réduire à une fois
	aux 5 secondes en cas d'inactivité.
- Publication à interval fixe de x minutes en cas d'inactivité. La valeur de x
	sera à déterminer, probablement 10 à 30 minutes. Programmabe à distance.
