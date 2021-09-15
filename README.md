# User Interface for telepresence robot

Questa repository contiene il codice del software sviluppato per la Tesi di Laurea triennale, in Ingegneria informatica, di Filippo Bianco.

## Istruzioni
Per avviare il server:
```
node server/app.js
```
Per avviare l'app nel computer connesso al robot:
1. nel computer connesso al robot, installare il pacchetto ROS rosbridge_suite: `sudo apt-get install ros-<rosdistro>-rosbridge-server`
2. una volta installato, porlo nella path di sistema: `source /opt/ros/<rosdistro>/setup.bash`
3. avviare rosbridge: `roslaunch rosbridge_server rosbridge_websocket.launch`
4. avviare l'applicazione
5. prima di effettuare l'accesso, compilare i campi relativi al topic delle immagini compresse, al topic dei comandi di velocità e all'indirizzo del WebSocket server locale aperto da rosbridge (tipicamente ws://0.0.0.0:9090, lo si può trovare nella console in seguito al lancio di rosbridge_suite).
