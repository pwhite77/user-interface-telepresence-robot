const httpolyglot = require('httpolyglot');
var sqlite3 = require('sqlite3').verbose();
var db = new sqlite3.Database('storage');
const bcrypt = require('bcrypt');
const path = require('path');
const express = require('express');
const server = express();

const credentials = {
    /* Le credenziali del certificato sono necessarie per testare la connessione HTTPS su localhost.
       Senza HTTPS, moltissimi browser impediscono l'utilizzo di API quali getUserMedia e RTCPeerConnection.
       La private key qui sotto e il corrispondente certificate (cert) sono emessi per filippob.ga.
       Sull'hosting Heroku (https://filipportc.herokuapp.com) non sono utilizzati poiché è già disponibile
       un altro certificato valido per tale dominio.
     */
    key: "-----BEGIN PRIVATE KEY-----\n" +
        "MIIEvgIBADANBgkqhkiG9w0BAQEFAASCBKgwggSkAgEAAoIBAQDFr9DcHc/cSjoa\n" +
        "FZZPxyGgMtCt4+UXRyyN06XfS5a678C5zSLoXKlBCVl71bClx9qAgpoIIpKz1V6Q\n" +
        "QLDXM68oQEkYh7nMruOm6fbW2PZzveVe611GfDtfFOO8kU2qLf70AVmCriUBZpMb\n" +
        "Sr8wBuq8v45sspvVfjCOhP800Cc3bFugGaZbNxz4kpr9/YLWllp/AZS9O2xNWZzo\n" +
        "D1D9b494YkVq8wtSiy0HfPwdO6Vt9o82uobeI4IYXfWc/YZVV/HGf+59xKgUcCq6\n" +
        "lAOqYVHi2aFDTkzfLOLGRQFRXyrE9XKXNRNvouk25qQImQPpQqiAYIbwWtvwNHfA\n" +
        "F6FvR2GpAgMBAAECggEASFuWIzt2PbedlLaaEhFdKXnwD/X+gIq4sCDr8dOFFF91\n" +
        "N4zyXgKsoPV/H2iUA9onDCrBnoCpGdHbjwlesSZl0mHVX3kudND+2rWAeBtp+etj\n" +
        "7V5RAd+vC5pl0TuOJeA4Fa/4x3BecyjMZ4zwde/SM8wZoYeuqnJJ66CamQYnlI0b\n" +
        "Z9FmYjW04iiPeNYB/94+uEdeYGJ5v2iENxIP6/O29jKw0htNo1bQ2SbEVMvxP/2t\n" +
        "sG1ff6aNZIZagvv+gK/johpMwFS/e1cgWPiRX38x7/9ctlndLXODdijxwZmDB6xg\n" +
        "qjzngp4J/6O+iVhRmw9Sn7dpZUbOsx7zj8HSU+OhQQKBgQD4w7WuKzyRCA4BawTl\n" +
        "Eahy93jthGaifTK3z2S48c5JY40JZVDukbDHQJr+V86S3HAz3UP6YIMb7io8cABh\n" +
        "H1gHgYYvIcEBO7w7wAD5+taBNnNJazDRRDT7d9F+d7UNOgA0NPehqrItDvdVNflp\n" +
        "qnZTZ0VXr5LEfY5ebPi+S7c0nQKBgQDLb8iatHBKM5C/owx6iBimcirhyuaGFv1n\n" +
        "+3GH40svAmqyFqoZAuL50brJskyFvc3Bhae55lxtuwCi1evXQV8w+3I5rNlvb792\n" +
        "bURsl9gmhxObMAZkxkoRdt9XiZbxF4kyXEolk4insMYiKNKgOm8gY31a6mEAE7Fs\n" +
        "4QA3C5glfQKBgQDKyjgj6GkyTZ+lNknCNvfb6LNy5FkKowciYnXYcn5Yw0eo3ifw\n" +
        "o2G5vIfdSVVD0WHCVeDhjaWzsh5KMH+OrQ+E5uitKVX+HGUhTC8/mWUd5nZq5m/g\n" +
        "WDrcuFtWPl2gj6S06mzoJI+lPasQkrRkmCpnaStVBPKfLT9O6ISKFI0f3QKBgQCv\n" +
        "KGj1YsSH7swDZDR7T7DBpuzrCdLfwu2eWO1QTdBPAmqxOThtXoHlDMIhpWWjhV+h\n" +
        "7x0Rv5j3VZpfmYZ9CKkYOHcJ15xDCVk1czEFutt9mLG13Wyz2dKuJMZ5zVpr7JLR\n" +
        "DIs+vT9c+qQthy6KjKVfoqms3L513X7XOwb/Xfk61QKBgFSC930SHAWvt4rzo6HF\n" +
        "UeKOyTsiBv61V5Eeg8UOkhPH/n66D/QWo8ZHQRciCXfoiuF/NK8m86tk1jVOQyO5\n" +
        "R5JWusGbenp+NgH+LGkfjyuezeVZa3qQo6/1jTGgnLefWcDXI6CDeTofnTCgg76H\n" +
        "01tF938SgfX5iqd7gcfPTKsS\n" +
        "-----END PRIVATE KEY-----",
    cert: "-----BEGIN CERTIFICATE-----\n" +
        "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\n" +
        "-----END CERTIFICATE-----"
}

// Database
// Create tables if needed.
db.serialize(function() {
   db.run("CREATE TABLE IF NOT EXISTS `users` (\n" +
       "\t`id` INT(255) NOT NULL,\n" +
       "\t`username` VARCHAR(255) NOT NULL,\n" +
       "\t`password` VARCHAR(255) NOT NULL,\n" +
       "\t`role` INT(1) NOT NULL,\n" +
       "\t`signup` TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,\n" +
       "\tPRIMARY KEY (`id`)\n" +
       ");");
   db.run("CREATE TABLE IF NOT EXISTS `rooms` (\n" +
       "\t`name` VARCHAR(255) NOT NULL,\n" +
       "\t`password` VARCHAR(255) NOT NULL,\n" +
       "\t`locked` INT(1) DEFAULT '0',\n" +
       "\t`created` TIMESTAMP DEFAULT CURRENT_TIMESTAMP,\n" +
       "\tPRIMARY KEY (`name`)\n" +
       ");");
});

// Routing
server.use(express.static(path.join(__dirname, '..','public')));
server.use(express.static(path.join(__dirname, '..','node_modules')));
server.use(express.urlencoded({extended: true}));

server.post('/api/username/:socketID', function (req, res) {
    if (!peers[req.params.socketID]) {
        res.send({"success": false});
    }
    res.send({"success": true, "username": peers[req.params.socketID].handshake.query.userName});
});

server.post('/api/role', function (req, res) {
    let username = req.body.username;
    db.get("SELECT `role` FROM `users` WHERE `username`=?", [username], function(err, row) {
        if (row) {
            res.send({"success": true, "role": row.role});
        } else {
            res.send({"success": false});
        }
    });
});

server.post("/api/room", function(req, res) {
    let roomName = req.body.roomName;
    let roomPassword = req.body.roomPassword;
    if (!roomPassword || !roomName) {
        res.send({"success": false});
    }
    db.get("SELECT * FROM `rooms` WHERE `name`=?", [roomName], function(err, row) {
       if (!row) {
           bcrypt.hash(roomPassword, 10, function(err, hash) {
               db.run("INSERT INTO `rooms`(`name`, `password`) VALUES(?, ?)", [roomName, hash], function(err) {
                   res.send({"success": true, "roomName": roomName});
               });
           });
       } else {
           // Controlla password.
           bcrypt.compare(roomPassword, row.password, function (err, match) {
               if (match) {
                   res.send({"success": true, "roomName": roomName});
               } else {
                   res.send({"success": false});
               }
           });
       }
    });
});

server.post("/api/login", function(req, res) {
    // TODO per rendere effettiva l'autenticazione, rimuovere la riga successiva a questa
    //res.send({"success": true, "username": req.body.username, "role": 0});
    //return;
    let username = req.body.username;
    db.get("SELECT * FROM `users` WHERE `username`=?", [username], function(e, row) {
        if (row) {
            bcrypt.compare(req.body.password, row.password, function(err, match) {
               if (match) {
                   res.send({"success": true, "username": username, "role": row.role});
               } else {
                   res.send({"success": false});
               }
            });
        } else {
            res.send({"success": false});
        }
   });
});

const httpsServer = httpolyglot.createServer(credentials, server);
const io = require('socket.io')(httpsServer);

peers = {};

io.on('connect', (socket) => {
    let roomName = socket.handshake.query.roomName;
    let userName = socket.handshake.query.userName;

    // Initiate the connection process as soon as the client connects
    peers[socket.id] = socket;

    for(let id in peers) {
        // Salta me stesso e i peer che non sono nella stessa stanza.
        if(id === socket.id || peers[id].handshake.query.roomName !== roomName) {
            continue;
        }
        peers[id].emit("prepare_connection", {socketID: socket.id, roomName: roomName});
    }

    /**
     * relay a peerconnection signal to a specific socket
     */
    socket.on("signal", data => {
        if (data.roomName !== socket.handshake.query.roomName) {
            return;
        }
        if(!peers[data.socketID]) {
            return;
        }
        peers[data.socketID].emit("signal", {
            socketID: socket.id,
            signal: data.signal,
            roomName: socket.handshake.query.roomName
        });
    });

    /**
     * remove the disconnected peer connection from all other connected clients
     */
    socket.on("disconnect", () => {
        socket.broadcast.emit("kickout", { socketID: socket.id, roomName: socket.handshake.query.roomName });
        delete peers[socket.id];
    });

    /**
     * Send message to client to initiate a connection
     * The sender has already setup a peer connection receiver
     */
    socket.on("connection_prepared", data => {
        if (data.roomName !== socket.handshake.query.roomName) {
            return;
        }
        let initSocketID = data.socketID;
        peers[initSocketID].emit("connection_prepared", {socketID: socket.id, roomName: socket.handshake.query.roomName});
    });

    socket.on("message", data => {
        for (let id in peers) {
            if (peers[id].handshake.query.roomName !== data.roomName) {
                // Questo messaggio non è per questo peer!
                continue;
            }
            peers[id].emit("message", data);
        }
    });

    socket.on("robot_movement_command", data => {
        for (let id in peers) {
            if (peers[id].handshake.query.roomName !== data.roomName) {
                continue;
            }
            peers[id].emit("robot_movement_command", data);
        }
    });

    socket.on("robot_image", data => {
        for (let id in peers) {
            if (peers[id].handshake.query.roomName !== data.roomName) {
                continue;
            }
            peers[id].emit("robot_image", data);
        }
    });
});

httpsServer.listen(process.env.PORT, () => {
    console.log(`Server listening. Port ${process.env.PORT}`);
})
