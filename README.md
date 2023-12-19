# Installation

L'utilisation du code nécessite un certain nombre de bibliothèques Python. Pour les
installer sur votre environnement, nous vous invitons à suivre les étapes suivantes.

# Etape 1 : Environnement (Ubuntu 22.04)

Pour cette étape, vous avez 2 choix :

```
Installer Ubuntu 22.04 sur votre machine
Installer Ubuntu 22.04 sur une machine virtuelle
```
Ubuntu 22.04.2 LTS Image

# Etape 2 : Installer les bibliothèques Python

**Attention : assurez-vous d'avoir placé le dossier aquabot_theboys dans
/home/user/vrx_ws/src/**

Pour faciliter l'installation des bibliothèques, nous avons créé un fichier
'requirements.txt' listant toutes les bibliothèques nécessaires.

```
cd ~/vrx_ws/src/aquabot_theboys/resource/
pip install -r requirements.txt
```
La version de setuptools utilisée pour construire l’environnement ROS est la 58.2.0,
or cette version n’est pas compatible avec notre nœud de reconnaissance d’objets qui
utilise au minimum la version 65.5.1.

Pour éviter de rencontrer ce problème, voici la marche à suivre :

**A faire avant chaque commande colcon build**

```
pip install setuptools==58.2.
```
Le modèle de reconnaissance d’objet récupérera la dernière version de setuptools sur
internet au lancement du nœud.

# Etape 3 : Lancement de l'environnement

Notre lanceur devra être lancé après le lancement de l’environnement Gazebo. Voici la
commande nécessaire :

```
ros2 launch aquabot_theboys aquabot_theboys.launcher.py
```


