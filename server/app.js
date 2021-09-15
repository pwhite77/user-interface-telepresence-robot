const httpolyglot = require('httpolyglot');
var sqlite3 = require('sqlite3').verbose();
var db = new sqlite3.Database('storage');
const bcrypt = require('bcrypt');
const path = require('path');
const express = require('express');
const server = express();

const credentials = {
    // Credenziali ommesse nella repository.
    key: "-----BEGIN PRIVATE KEY-----",
    cert: "-----BEGIN CERTIFICATE-----"
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
    console.log(`Started on port: ${process.env.PORT}`);
});
