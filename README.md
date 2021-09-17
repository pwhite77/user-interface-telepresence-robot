# User Interface for telepresence robot

Questa repository contiene il codice del software sviluppato per la Tesi di Laurea triennale, in Ingegneria informatica, di Filippo Bianco.

## Istruzioni
Per avviare il server:
```
node server/app.js
```

**NB:** per la [Demo dell'interfaccia](https://filipportc.herokuapp.com) è stato scelto come ambiente server Heroku. Per caricare l'applicazione su Heroku è sufficiente collegare il proprio account Heroku con l'account Github. Il deploy sarà effettuato in automatico, in base alle istruzioni specificate nel file `package.json`

Per avviare l'app nel computer connesso al robot:
1. nel computer connesso al robot, installare il pacchetto ROS rosbridge_suite: `sudo apt-get install ros-<rosdistro>-rosbridge-server`
2. una volta installato, porlo nella path di sistema: `source /opt/ros/<rosdistro>/setup.bash`
3. avviare rosbridge: `roslaunch rosbridge_server rosbridge_websocket.launch`
4. avviare l'applicazione
5. prima di effettuare l'accesso, compilare i campi relativi al topic delle immagini compresse, al topic dei comandi di velocità e all'indirizzo del WebSocket server locale aperto da rosbridge (tipicamente ws://0.0.0.0:9090, lo si può trovare nella console in seguito al lancio di rosbridge_suite).

**NB:** nel web-browser del computer connesso al robot è necessario disabilitare la web-security: [Firefox](https://support.mozilla.org/en-US/kb/mixed-content-blocking-firefox#w_unblock-mixed-content), [Chrome](https://support.google.com/chrome/answer/114662?hl=en&co=GENIE.Platform=Desktop#zippy=%2Cpermissions-that-can-be-changed).

![Demo dell'interfaccia utente](https://github.com/pwhite77/user-interface-telepresence-robot/raw/main/move.png)
