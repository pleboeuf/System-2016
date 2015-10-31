# Logiciel pour érablière. System-2016

Logiciel pour une érablière utilisant différents capteurs basés sur les modules "Photon" et éventuellement "Electron" de la 
compagnie "Particle" (www.particle.io). Un logiciel pour la création d'un site web affichant ces informations sera ajouté dans les prochains mois.

Les capteurs effectuent les mesures et publient des événement dans le nuage. Le logiciel d'analyse "s'abonne" à ces événements
et accumule les diverses mesures dans une base de données. De plus il affiche l'état de la coulée et du système sous forme de page web et produira un rapport de l'activité.

Les capteurs sont équipés de mémoire alimenté par batterie et accululent jusqu'à 300 événements en cas de perte du réseau.
Les événement sont sérialisé pour s'assurer que l'on ne perdent pas de données et qu'elles sont dans le bon ordre.

- Capteur de distance pour mesurer le niveau des réservoirs et la position des valves de ces réservoirs.
- Capteur robuste de distance pour les réservoir externe et la position des valves de ces réservoirs.
- Capteur de marche / arrêt pour les pompes des relâcheur et optionellement de débitmêtres.
- Capteur de vide.
- Capteur de pression atmosphérique.
