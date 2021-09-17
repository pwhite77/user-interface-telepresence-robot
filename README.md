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

## Descrizione componenti

Nella cartella `server` è possibile trovare il file `app.js` che rappresenta il backend (scritto in Node.js) dell'applicazione.
All'interno della cartella `public` è invece possibile trovare le componenti di frontend:
1. `index.html` è la pagina principale, che contiene il markup HTML dell'interfaccia e include i vari file JavaScript che ne determinano il funzionamento. Tutti questi file sono organizzati all'interno della sottocartella `js`;
2. `main.js` contiene le funzionalità per la componente di videoconferencing, per la connessione al robot, per la gestione del movimento del robot, per la chat e per la connessione al WebSocket server;
3. `pages.js` contiene le funzionalità necessarie alla realizzazione della paginazione dell'interfaccia (durante il processo di login e di scelta della room);
4. `pose-estimation.js` contiene l'integrazione delle due librerie di pose estimation, PoseNet e MediaPipe;
5. `robot-moves.js` contiene le funzionalità per gestire graficamente l'interfaccia di comando dei movimenti del robot;
6. `visual-helpers.js` contiene delle funzionalità utili a gestire l'aspetto grafico dell'interfaccia.

![Demo dell'interfaccia utente](https://github.com/pwhite77/user-interface-telepresence-robot/raw/main/move.png)
