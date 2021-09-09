
async function getConnectedVideoDevices() {
    const devices = await navigator.mediaDevices.enumerateDevices();
    return devices.filter(device => device.kind === "videoinput");
}

function startLocalVideo() {
    if (userRole === 1) {
        /*

        FLOW ALTERNATIVO (poiché image_to_v4l2loopback non funziona):
        Se sono il robot:
            1) inizializzo rosBridge, ovvero la connessione con roslibjs
            2) mi abbono al topic dove il robot pubblica le immagini (il topic deve essere COMPRESSED)
            3) quando ricevo un'immagine dal robot, emetto agli altri peer connessi nella stanza un messaggio di tipo "robot_image"
            4) carico, nell'immagine con id "robotStream", l'immagine ricevuta nel topic
            5) quando gli altri peer ricevono un messaggio socket di tipo "robot_image", caricano nell'immagine con id "robotStream" l'immagine ricevuta tramite socket.

            Problema1: non essendo robotStream un elemento di tipo video, bisogna adattare "openBigger" e "restoreSmaller"
            Problema2: bisogna adattare il codice di PoseNet e MediaPipe per lavorare con immagine invece che con video.
            Problema3: se sono il robot, devo connettermi anche con l'audio oltre che con il video.

         */
        // Attivo l'audio
        let constraints = {
            "audio": true,
            "video": false
        }
        navigator.mediaDevices.getUserMedia(constraints).then(async stream => {

            document.querySelector("#localVideo").srcObject = stream;
            localStream = stream;

            await initializeSocketConnection();

            initializeRobotConnection();

        }).catch(async function(e) {
            // Se non c'è alcuna device disponibile, ignora l'audio.

            localStream = document.getElementById("mediaStreamUnavailableCanvas").captureStream();
            document.querySelector("#localVideo").srcObject = localStream;

            await initializeSocketConnection();

            initializeRobotConnection();

        });


    } else {
        let constraints = {
            "audio": true,
            "video": true
        }
        navigator.mediaDevices.getUserMedia(constraints).then(stream => {

            document.querySelector("#localVideo").srcObject = stream;
            localStream = stream;

            initializeSocketConnection();

        }).catch(function(e) {
            /* L'utente ha negato il consenso all'accesso alla videocamera/microfono.
               Oppure il dispositivo è sprovvisto di videocamera/microfono.
               Si può entrare comunque nella sessione ma gli altri partecipanti non vedranno nè sentiranno nulla.
             */
            alert("Potrai unirti alla room, ma non comunicare con gli altri partecipanti, né essere visto o sentito." +
                "Per permetterti agli altri di vederti e sentirti, attiva o collega webcam e microfono.");
            $("#localVideo").attr("src", null);
            localStream = null;
            initializeSocketConnection();
        });
    }
}

function initializeRobotConnection() {
    // Inizializza roslibjs
    rosBridge = new ROSLIB.Ros({
        url: $("#commandsServerUrl").val()
    });
    // Message handlers
    rosBridge.on("connection", function() {
        console.log("rosBridge connected successfully.");
    });
    rosBridge.on("error", function(error) {
        console.log("rosBridge has failed: ", error);
    });
    rosBridge.on("close", function() {
        console.log("rosBridge closed.");
    });
    // Inizializza imageListener, cioè un listener che riceve le immagini:
    imageListener = new ROSLIB.Topic({
        ros: rosBridge,
        name: $("#topicName").val(),
        messageType: "sensor_msgs/CompressedImage"
    });
    // Quando ricevo un'immagine...
    imageListener.subscribe(function(message) {
        // Rigiro agli altri peer
        let msg = {
            roomName: roomName,
            image: message.data
        }
        if (socket) {
            socket.emit("robot_image", msg);
        }
        document.getElementById("robotStream").src = "data:image/jpg;base64," + message.data;
    });
}

async function initializeSocketConnection() {
    socket = io("", {
        query: {
            "roomName": roomName,
            "userName": userName,
        }
    });

    socket.on("connect", function() {
        document.getElementById("localVideo").setAttribute("data-socket", socket.id);
        document.getElementById("localVideo").parentElement.setAttribute("id", socket.id);
        document.getElementById("localVideo").addEventListener("click", function() {
            openBigger(socket.id);
        });
    });

    socket.on("prepare_connection", async function(data) {
        if (data.roomName !== roomName) {
            return;
        }
        await userJoined(data.socketID, false);
        socket.emit("connection_prepared", { socketID: data.socketID, roomName: roomName });
    });

    socket.on("connection_prepared", async function(data) {
        if (data.roomName !== roomName) {
            return;
        }
        await userJoined(data.socketID, true);
    });

    socket.on("kickout", function(data) {
        if (data.roomName !== roomName) {
            return;
        }
        kickout(data.socketID);
    });

    socket.on("disconnect", function() {
        for (let socketID in peers) {
            if (peers[socketID].handshake.query.roomName !== roomName) {
                continue;
            }
            kickout(socketID);
        }
    });

    socket.on("signal", data => {
        if (data.roomName !== roomName) {
            return;
        }
        peers[data.socketID].signal(data.signal);
    });

    socket.on("message", data => {
       if (data.roomName !== roomName) {
           // Questo messaggio non è per questa room.
           // TODO verificare necessità di questo controllo perchè un controllo simile viene fatto anche server-side.
           return;
       }
       console.log("Message received");
       messageReceived(data);
    });

    socket.on("robot_image", data => {
        if (data.roomName !== roomName) {
            // Questo messaggio non è per questa room.
            // TODO verificare necessità di questo controllo perchè un controllo simile viene fatto anche server-side.
            return;
        }
        document.getElementById("robotStream").src = "data:image/jpg;base64," + data.image;
        if (poseEstimationInProgress) estimatePose(document.getElementById("robotStream"));
    });
}

function openRobotVideo() {
    // Chiudi gli altri flussi video ingranditi.
    restoreSmaller();
    $("#robotStreamContainer").hide();

    let img = $("#robotStream");
    // Remove video from parent.
    img.detach();

    img.css({
        "width": "auto",
        "max-width": "100%",
        "height": "100%",
    });

    img.appendTo("#main-video-cell");

    $("#restoreSmallerBtn").show();

    // Se io sono l'amministratore E il peer ingrandito è il robot:
    if (userRole == 0) {
        $("#button-up, #button-down").show();
        let top = 150 + ($("#main-video-cell img").first().innerHeight() / 2) - 39;
        $("#button-left").css({
            "left": Math.round($(window).width() / 5) + "px",
            "top": Math.round(top) + "px"
        });
        $("#button-right").css({
            "right": Math.round( $(window).width() / 5 ) + "px",
            "top": Math.round(top) + "px"
        });
        $("#button-left, #button-right, #speedSlider").show();
    }
}

function closeRobotVideo() {
    let img = $("#main-video-cell img").first();
    img.detach();
    img.css({
        "height": "100px",
        "width": "200px"
    });
    img.appendTo($("#robotStreamContainer"));
    $("#robotStreamContainer").show();
    $("#restoreSmallerBtn").hide();
    $("#button-up, #button-down, #button-left, #button-right, #speedSlider").hide();
}

function messageReceived(data) {
    let content = data.content;

    switch (content) {
        case "message":
            createMessage(data);
            break;
        default:
            console.error("Message content was not recognized.");
    }
}

function createMessage(data) {
    let fromUsername = data.fromUsername;
    if (data.type !== "text") {
        console.error("Only text messages are currently supported.");
        return;
    }
    let body = data.message;
    let timestamp = data.timestamp;
    let date = new Date(timestamp);
    let style = "message";
    if (data.fromSocket === socket.id) {
        style += " sent";
    }
    $("#chatMessages").append(
        "<div class=\"" + style + "\">\n" +
        "    <h1>From " + fromUsername + "</h1>\n" +
        "    <h2>" + body + "</h2>\n" +
        "    <h3>" + date.getHours() + ":" + date.getMinutes() + ":" + date.getSeconds() + "</h3>\n" +
        "</div>"
    );
    $("#chatMessages").scrollTop($("#chatMessages").height());
    if (data.fromSocket === socket.id) {
        return;
    }
    $("#messageNotificationTitle").text("New message from " + fromUsername);
    if (body.length > 50) {
        body = body.substring(0, 50) + "...";
    }
    $("#messageNotificationBody").text(body);
    $("#messageNotification").show();
    setTimeout(function(){ $("#messageNotification").fadeOut(); }, 3000);
}

function sendMessage(messageContent, messageType, messageBody) {
    /*
        "message": {
            "roomName": "...",
            "fromSocket": "...",
            "fromUsername": "...",
            "content": "signal|message",
            "type": "...", // Se content === "signal", allora "type" contiene il tipo di segnalazione (per es: move-neck).
                           // Se invece content === "message", allora "type" contiene il tipo di messaggio (text/audio/...).
                           // Per ora è implementato solo "type" === "text".
            "message": "...",
            "timestamp": "..."
        }
     */
    if (!(messageContent === "signal" || messageContent === "message")) {
        return;
    }
    let messageJson = {
        roomName: roomName,
        fromSocket: socket.id,
        fromUsername: userName,
        content: messageContent,
        type: messageType,
        message: messageBody,
        timestamp: Date.now()
    };
    socket.emit("message", messageJson);
}

function kickout(socketId) {
    let videosContainer = document.getElementById(socketId);
    if (videosContainer) {

        let video = videosContainer.children[1];

        video.srcObject.getTracks().forEach(function (track) {
            track.stop();
        });

        video.srcObject = null;
        document.querySelector("#connected-users-row").removeChild(videosContainer);
    }
    if (peers[socketId]) {
        peers[socketId].destroy();
    }
    delete peers[socketId];
}

async function userJoined(socketID, initiator) {
    peers[socketID] = new SimplePeer({
        initiator: initiator,
        stream: localStream,
        config: configuration
    });

    peers[socketID].on("signal", data => {
        socket.emit("signal", {
            signal: data,
            socketID: socketID,
            roomName: roomName
        })
    });

    peers[socketID].on("stream", async function(stream) {
        let usernameReq = await Promise.resolve(
            $.post("/api/username/" + socketID)
        );
        let username = "Unknown user";
        if (usernameReq.success) {
            username = usernameReq.username;
        }

        $("#connected-users tr").append("<td id='" + socketID + "'>" +
            "<p>" + username + "</p>" +
            "</td>");

        let userJoinedVideo = document.createElement("video");
        userJoinedVideo.setAttribute("data-socket", socketID);
        userJoinedVideo.srcObject = stream;
        userJoinedVideo.setAttribute("playsinline", true);
        userJoinedVideo.playsinline = true;
        userJoinedVideo.autoplay = true;
        userJoinedVideo.className = "miniature";
        userJoinedVideo.addEventListener("click", function() {
            openBigger(socketID);
        });

        $("#" + socketID).append(userJoinedVideo);

        /*let newVidContainer = document.createElement("div");
        newVidContainer.id = socket_id;
        newVidContainer.setAttribute("class", "cell");
        let usernameReq = await Promise.resolve(
            $.post("/api/username/" + socket_id)
        );
        let username = "Unknown user";
        if (usernameReq.success) {
            username = usernameReq.username;
        }

        newVidContainer.innerHTML += "<p>" + username + "</p>";
        let userJoinedVideo = document.createElement('video');
        userJoinedVideo.setAttribute("data-socket", socket_id);
        userJoinedVideo.srcObject = stream;
        userJoinedVideo.setAttribute("playsinline", true); // For iPhone Safari
        userJoinedVideo.playsinline = true;
        userJoinedVideo.autoplay = true;
        userJoinedVideo.className = "miniature";
        userJoinedVideo.addEventListener("click", function() {
            openBigger(socket_id);
        });
        newVidContainer.appendChild(userJoinedVideo);
        document.getElementById("videosContainer").appendChild(newVidContainer);*/
    });

}

function openBigger(socketID) {
    if (!socketID) {
        return;
    }

    if ( $("#main-video-cell video").length ) {
        restoreSmaller();
    }

    $("#" + socketID).hide();

    let video = $("#" + socketID + " > video");
    // Remove video from parent.
    video.detach();

    video.css({
        "width": "auto",
        "height": "100%",
    });

    video.appendTo("#main-video-cell");

    /*if (socketID === socket.id) {
        // Si attiva solo sul peer locale
        let videoWidth = $("#main-video-row video").first().innerWidth();
        let videoHeight = $("#main-video-row video").first().innerHeight();
        canvas.width = videoWidth;
        canvas.height = videoHeight;
        document.getElementById("localVideo").width = videoWidth;
        document.getElementById("localVideo").height = videoHeight;
        canvas.style.width = videoWidth + "px";
        canvas.style.height = videoHeight + "px";
        canvas.style.left = Math.round(($("#main-video-row").innerWidth() / 2) - videoWidth / 2) + "px";
        canvas.style.display = "block";

        // Quale algoritmo di pose estimation devo attivare? 1 per MediaPipe, 0 per PoseNet
        let poseEstimationAlg = window.location.search === "?mp" ? 1 : 0;
        if (!poseEstimationAlg) {
            if (!poseNetEstimationInProgress) {
                detectPoseInRealTime(document.getElementById("localVideo"));
            }
        } else {
            if (!mediaPipeEstimationInProgress) {
                //startPoseEstimation(document.getElementById("localVideo"));
                startPoseEstimation(localStream);
            }
        }
    }*/

    $("#restoreSmallerBtn").show();
}

function restoreSmaller() {
    closeRobotVideo();
    let video = $("#main-video-cell video").first();
    video.detach();
    video.css({
        "height": "100px",
        "width": "auto"
    });
    video.appendTo( $("#" + video.attr("data-socket")) );
    $("#" + video.attr("data-socket")).show();
    $("#restoreSmallerBtn").hide();
    $("#button-up, #button-down, #button-left, #button-right").hide();

    /*let video = $("body > video").first();
    video.detach();
    video.css({
        "position": "static",
        "height": "100px",
        "width": "auto"
    });
    video.appendTo( $("#"+video.attr("data-socket")) );
    $("#"+video.attr("data-socket")).show();
    $("#restoreSmallerBtn").hide();
    $("#button-up, #button-down, #button-left, #button-right").hide();*/
}

function changeCamera(newDeviceId) {
    let constraints = {
        "audio": false, //TODO activate audio
        "video": {
            "deviceId": newDeviceId
        }
    };

    localStream.getTracks().forEach(function (track) {
        track.stop();
    });

    localVideo.srcObject = null;
    navigator.mediaDevices.getUserMedia(constraints).then(stream => {
        // Notifica gli altri socket che ho cambiato telecamera
        for (let socketId in peers) {
            let tracks = peers[socketId].streams[0].getTracks();
            for (let i = 0; i < tracks.length; i++) {
                let tracks2 = stream.getTracks();
                for (let j = 0; j < tracks2.length; j++) {
                    if (tracks[i].kind === tracks2[j].kind) {
                        peers[socketId].replaceTrack(tracks[i], tracks2[j], peers[socketId].streams[0]);
                        break;
                    }
                }
            }
        }

        /*for (let socket_id in peers) {
            for (let index in peers[socket_id].streams[0].getTracks()) {
                for (let index2 in stream.getTracks()) {
                    if (peers[socket_id].streams[0].getTracks()[index].kind === stream.getTracks()[index2].kind) {
                        peers[socket_id].replaceTrack(peers[socket_id].streams[0].getTracks()[index], stream.getTracks()[index2], peers[socket_id].streams[0])
                        break;
                    }
                }
            }
        }*/

        localStream = stream;
        localVideo.srcObject = stream;

        updateButtons();
    })
}

function hangup() {
    if (localStream) {
        localStream.getTracks().forEach(function (track) {
            track.stop();
        })

        document.querySelector("#localVideo").srcObject = null;
    }

    for (let socketId in peers) {
        kickout(socketId);
    }

    window.location.reload();
}

function toggle(audio) {
    let tracks = null;
    if (audio) {
        tracks = localStream.getAudioTracks();
    } else {
        tracks = localStream.getVideoTracks();
    }

    for (let i = 0; i < tracks.length; i++) {
        if (tracks[i].enabled) {
            tracks[i].enabled = false;
            if (audio) {
                $("#toggleMuteButton").html("<i class=\"bi bi-mic-fill\"></i>");
            } else {
                $("#toggleVideoButton").html("<i class=\"bi bi-camera-video-fill\"></i>");
            }
        } else {
            tracks[i].enabled = true;
            if (audio) {
                $("#toggleMuteButton").html("<i class=\"bi bi-mic-mute-fill\"></i>");
            } else {
                $("#toggleVideoButton").html("<i class=\"bi bi-camera-video-off-fill\"></i>");
            }
        }
    }
}

function updateButtons() {
    let audioTracks = localStream.getAudioTracks();
    for (let i = 0; i < audioTracks.length; i++) {
        if (audioTracks[i].enabled) {
            $("#toggleMuteButton").html("<i class=\"bi bi-mic-fill\"></i>");
        } else {
            $("#toggleMuteButton").html("<i class=\"bi bi-mic-mute-fill\"></i>");
        }
    }

    let videoTracks = localStream.getVideoTracks();
    for (let i = 0; i < videoTracks.length; i++) {
        if (videoTracks[i].enabled) {
            $("#toggleVideoButton").html("<i class=\"bi bi-camera-video-fill\"></i>");
        } else {
            $("#toggleVideoButton").html("<i class=\"bi bi-camera-video-off-fill\"></i>");
        }
    }
}
